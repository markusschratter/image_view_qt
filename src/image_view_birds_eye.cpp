// birds_eye_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

/**
 * BirdsEyeNode
 * - Subscribes to a compressed camera image and its CameraInfo
 * - Looks up the transform camera_frame -> target_frame
 * - Builds and caches remap maps (u,v) to warp the image onto a flat ground plane
 * - Publishes a BGR8 raw Image in target_frame (top-down / bird's-eye view)
 *
 * Parameters:
 *   image_topic        (string) input compressed image topic (default: /sensing/camera/front/image_raw/compressed)
 *   camera_info_topic  (string) camera info topic (default: /sensing/camera/front/camera_info)
 *   camera_frame       (string) name of the camera frame (default: camera_front)
 *   target_frame       (string) name of the output / ground plane frame (default: base_link)
 *   output_topic       (string) output raw image topic (default: /birds_eye/image_raw)
 *   output_size        (int[2]) [width, height] of BEV image (default: [800, 600])
 *   ground_x_min       (double) near distance (forward, meters) (default: 0.0)
 *   ground_x_max       (double) far  distance (forward, meters) (default: 20.0)
 *   ground_y_min       (double) left bound  (meters) (default: -5.0)
 *   ground_y_max       (double) right bound (meters) (default:  5.0)
 */
class BirdsEyeNode : public rclcpp::Node {
public:
  BirdsEyeNode() : rclcpp::Node("birds_eye_view_node"),
                   tf_buffer_(this->get_clock()),
                   tf_listener_(tf_buffer_) {
    // Parameters (input/output topics and geometry)
    image_topic_       = this->declare_parameter<std::string>("image_topic", "/sensing/camera/front/image_raw/compressed");
    camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/sensing/camera/front/camera_info");
    camera_frame_      = this->declare_parameter<std::string>("camera_frame", "camera_front");
    target_frame_      = this->declare_parameter<std::string>("target_frame", "base_link");
    output_topic_      = this->declare_parameter<std::string>("output_topic", "/sensing/camera/front/birds_eye_image");

    auto out_size      = this->declare_parameter<std::vector<int64_t>>("output_size", {1920, 1200});
    out_w_ = static_cast<int>(out_size.size()>0 ? out_size[0] : 800);
    out_h_ = static_cast<int>(out_size.size()>1 ? out_size[1] : 600);

    ground_x_min_ = this->declare_parameter<double>("ground_x_min", 0.0);
    ground_x_max_ = this->declare_parameter<double>("ground_x_max", 20.0);
    ground_y_min_ = this->declare_parameter<double>("ground_y_min", -5.0);
    ground_y_max_ = this->declare_parameter<double>("ground_y_max",  5.0);

    // Sensor QoS is appropriate for camera streams
    auto sensor_qos = rclcpp::SensorDataQoS();

    // Publisher: raw BEV image (BGR8)
    pub_img_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

    // Subscribers: CameraInfo and compressed image
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos, std::bind(&BirdsEyeNode::cameraInfoCb, this, _1));

    sub_img_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_, sensor_qos, std::bind(&BirdsEyeNode::imageCb, this, _1));

    RCLCPP_INFO(get_logger(), "Bird's Eye View node initialized");
  }

private:

  // Update intrinsics from CameraInfo and invalidate maps if they were built
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    cam_model_.fromCameraInfo(*msg);
    fx_ = cam_model_.fx();
    fy_ = cam_model_.fy();
    cx_ = cam_model_.cx();
    cy_ = cam_model_.cy();

    camera_info_received_ = true;
    intrinsics_version_++;
    map_ready_ = false;  // force remap map rebuild
  }

  // Decode compressed image, refresh maps if TF changed, remap to BEV and publish
  void imageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Waiting for CameraInfo…");
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      // We want the transform from camera_frame into target_frame
      // lookupTransform(target, source, time)
      tf = tf_buffer_.lookupTransform(target_frame_, camera_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    // Rebuild remap maps if intrinsics changed or TF changed
    if (!map_ready_ || tfChanged(tf)) {
      buildRemapMaps(tf);
    }

    // Decode JPEG/PNG
    cv::Mat bev;
    try {
      cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1, const_cast<uint8_t*>(msg->data.data()));
      cv::Mat image = cv::imdecode(buf, cv::IMREAD_COLOR);
      if (image.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to decode compressed image.");
        return;
      }

      // Apply precomputed mapping
      cv::remap(image, bev, map_u_, map_v_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception during image decode/remap: %s", e.what());
      return;
    }

    // Publish BEV image (BGR8) in target_frame with original stamp
    auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bev).toImageMsg();
    out_msg->header.stamp = msg->header.stamp;
    out_msg->header.frame_id = target_frame_;
    pub_img_->publish(*out_msg);
  }

  // ==== Helpers ====

  // Detects whether TF has changed since last used (translation/rotation thresholds)
  bool tfChanged(const geometry_msgs::msg::TransformStamped &tf) {
    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;

    if (!have_last_tf_) {
      last_tf_ = tf;
      have_last_tf_ = true;
      return true;
    }
    const auto &lt = last_tf_.transform.translation;
    const auto &lq = last_tf_.transform.rotation;

    const double eps_t = 1e-6;
    const double eps_q = 1e-9;

    bool trans_changed =
      std::fabs(t.x - lt.x) > eps_t ||
      std::fabs(t.y - lt.y) > eps_t ||
      std::fabs(t.z - lt.z) > eps_t;

    bool rot_changed =
      std::fabs(q.x - lq.x) > eps_q ||
      std::fabs(q.y - lq.y) > eps_q ||
      std::fabs(q.z - lq.z) > eps_q ||
      std::fabs(q.w - lq.w) > eps_q;

    if (trans_changed || rot_changed) {
      last_tf_ = tf;
      return true;
    }
    return false;
  }

  // Builds the u,v remap maps for cv::remap given current intrinsics and TF
  void buildRemapMaps(const geometry_msgs::msg::TransformStamped &tf) {
    // Allocate maps (height x width)
    map_u_.create(out_h_, out_w_, CV_32FC1);
    map_v_.create(out_h_, out_w_, CV_32FC1);

    // Create linspace along forward (X) and lateral (Y) directions on ground plane Z=0
    cv::Mat Xcol(out_h_, 1, CV_32F);
    cv::Mat Yrow(1,     out_w_, CV_32F);
    const float dx = (out_h_ > 1) ? static_cast<float>((ground_x_max_ - ground_x_min_) / (out_h_ - 1)) : 0.f;
    const float dy = (out_w_ > 1) ? static_cast<float>((ground_y_max_ - ground_y_min_) / (out_w_ - 1)) : 0.f;
    for (int i=0; i<out_h_; ++i) Xcol.at<float>(i,0) = static_cast<float>(ground_x_min_) + i*dx;
    for (int j=0; j<out_w_; ++j) Yrow.at<float>(0,j) = static_cast<float>(ground_y_min_) + j*dy;

    cv::Mat Xw, Yw;
    cv::repeat(Xcol, 1, out_w_, Xw);
    cv::repeat(Yrow, out_h_, 1, Yw);

    // TF parts: we have T_world_cam (target <- camera), i.e., R_wc and t_wc
    const auto &tt = tf.transform.translation;
    const auto &rq = tf.transform.rotation;

    tf2::Quaternion q(rq.x, rq.y, rq.z, rq.w);
    tf2::Matrix3x3 R_wc(q); // camera -> world
    // For world -> camera we use R_cw = R_wc^T and t_cw = -R_cw * t_wc.
    // We'll apply Pc = R_cw * (Pw - t_wc). (Z_w = 0)
    const float tx = static_cast<float>(tt.x);
    const float ty = static_cast<float>(tt.y);
    const float tz = static_cast<float>(tt.z);

    // Elements of R_wc
    const float a = static_cast<float>(R_wc[0][0]);
    const float d = static_cast<float>(R_wc[1][0]);
    const float g = static_cast<float>(R_wc[2][0]);

    const float b = static_cast<float>(R_wc[0][1]);
    const float e = static_cast<float>(R_wc[1][1]);
    const float h = static_cast<float>(R_wc[2][1]);

    const float c = static_cast<float>(R_wc[0][2]);
    const float f = static_cast<float>(R_wc[1][2]);
    const float i = static_cast<float>(R_wc[2][2]);

    // (Pw - t_wc)
    cv::Mat Xm, Ym;
    Xw.convertTo(Xm, CV_32F);
    Yw.convertTo(Ym, CV_32F);
    Xm -= tx;
    Ym -= ty;
    const float mz = -tz;

    // Pc = R_cw * (Pw - t_wc) = R_wc^T * (...)
    cv::Mat Xc = Xm*a + Ym*d; Xc += (g*mz);
    cv::Mat Yc = Xm*b + Ym*e; Yc += (h*mz);
    cv::Mat Zc = Xm*c + Ym*f; Zc += (i*mz);

    // Clamp Z to avoid division by zero or negative depths
    constexpr float eps = 1e-6f;
    cv::max(Zc, eps, Zc);

    // Perspective projection using camera intrinsics
    cv::Mat U, V, tmp;
    cv::divide(Xc, Zc, tmp);
    U = tmp * static_cast<float>(fx_);
    U += static_cast<float>(cx_);

    cv::divide(Yc, Zc, tmp);
    V = tmp * static_cast<float>(fy_);
    V += static_cast<float>(cy_);

    // Copy to CV_32FC1 maps for cv::remap
    U.convertTo(map_u_, CV_32FC1);
    V.convertTo(map_v_, CV_32FC1);

    map_ready_ = true;
    RCLCPP_DEBUG(get_logger(), "Updated remap maps (intrinsics v%u).", intrinsics_version_);
  }


  std::string image_topic_, camera_info_topic_, camera_frame_, target_frame_, output_topic_;

  int out_w_{800}, out_h_{600};

  double ground_x_min_{0.0}, ground_x_max_{20.0}, ground_y_min_{-5.0}, ground_y_max_{5.0};

  image_geometry::PinholeCameraModel cam_model_;
  bool  camera_info_received_{false};
  uint32_t intrinsics_version_{0};
  double fx_{0.}, fy_{0.}, cx_{0.}, cy_{0.};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::TransformStamped last_tf_;
  bool have_last_tf_{false};

  bool map_ready_{false};
  cv::Mat map_u_, map_v_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BirdsEyeNode>());
  rclcpp::shutdown();
  return 0;
}
