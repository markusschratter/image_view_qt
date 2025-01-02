#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <QApplication>
#include <QGuiApplication>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QScreen>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>


class QtWindow : public QLabel {
 public:
  explicit QtWindow(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr)
      : QLabel(parent), node_(node) {
    initializeParameters();
    setupWindow();
    setupROSInterfaces();
    process_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&QtWindow::process, this));
  }

 private:
  void initializeParameters() {
    image_topic_ = declareAndGet<std::string>("image_topic", "/image_raw");
    compressed_image_topic_ = declareAndGet<std::string>("compressed_image_topic", 
                                                          "/image_raw/compressed");
    use_compressed_ = declareAndGet<bool>("use_compressed", false);

    visualization_service_ = declareAndGet<std::string>("visualization_service", "/visualization");
    visualization_topic_ = declareAndGet<std::string>("visualization_topic", "/visualization");


    screen_index_ = declareAndGet<int>("screen_index", 0);
    position_ = declareAndGet<std::string>("position", "top:center");
    pos_x_ = declareAndGet<int>("pos_x", 0);
    pos_y_ = declareAndGet<int>("pos_y", 0);
    width_ = declareAndGet<int>("width", 800);
    height_ = declareAndGet<int>("height", 600);
    roi_ = cv::Rect(declareAndGet<int>("roi_x", 0), declareAndGet<int>("roi_y", 0),
                    declareAndGet<int>("roi_width", -1), declareAndGet<int>("roi_height", -1));

    stay_on_top_ = declareAndGet<bool>("stay_on_top", true);
    show_startup_ = declareAndGet<bool>("show_startup", true);
  }


  void setupWindow() {
    QList<QScreen*> screens = QGuiApplication::screens();
    QScreen* selected_screen = (screen_index_ >= 0 && screen_index_ < screens.size())
                                   ? screens[screen_index_]
                                   : QGuiApplication::primaryScreen();

    QRect screen_geometry = selected_screen->geometry();
    int screen_width = screen_geometry.width();
    int screen_height = screen_geometry.height();

    int x = 0, y = 0;
    if (position_ == "top:left") {
      x = 0;
      y = 0;
    } else if (position_ == "top:center") {
      x = screen_width / 2 - width_ / 2;
      y = 0;
    } else if (position_ == "top:right") {
      x = screen_width - width_;
      y = 0;
    } else if (position_ == "middle:left") {
      x = 0;
      y = screen_height / 2 - height_ / 2;
    } else if (position_ == "middle:center" || position_ == "center") {  // center is default
      x = screen_width / 2 - width_ / 2;
      y = screen_height / 2 - height_ / 2;
    } else if (position_ == "middle:right") {
      x = screen_width - width_;
      y = screen_height / 2 - height_ / 2;
    } else if (position_ == "bottom:left") {
      x = 0;
      y = screen_height - height_;
    } else if (position_ == "bottom:center") {
      x = screen_width / 2 - width_ / 2;
      y = screen_height - height_;
    } else if (position_ == "bottom:right") {
      x = screen_width - width_;
      y = screen_height - height_;
    } else {
      // Custom or fallback position
      x = pos_x_;
      y = pos_y_;
    }

    move(x, y);

    setWindowFlags(stay_on_top_ ? Qt::Window | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint
                                : Qt::Window | Qt::FramelessWindowHint);
    resize(width_, height_);
    setPalette(QPalette(Qt::black));

    if (show_startup_) {
      show();
    } else {
      hide();
    }
  }

  void setupROSInterfaces() {

    if (use_compressed_) {
      compressed_image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
          compressed_image_topic_, 10,
          std::bind(&QtWindow::compressedImageCallback, this, std::placeholders::_1));
    } else {
      image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
          image_topic_, 10, std::bind(&QtWindow::imageCallback, this, std::placeholders::_1));
    }
    
    visualize_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        visualization_topic_, 10,
        std::bind(&QtWindow::visualizeTopicCallback, this, std::placeholders::_1));

    visualize_srv_ = node_->create_service<std_srvs::srv::SetBool>(
        visualization_service_,
        std::bind(&QtWindow::visualizeServiceCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    diagnostic_pub_ =
        node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics2", 10);
    last_image_received_time_ = node_->get_clock()->now();
  }

  template <typename T>
  T declareAndGet(const std::string& name, const T& default_value) {
    node_->declare_parameter(name, default_value);
    return node_->get_parameter(name).get_value<T>();
  }


  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_image_received_time_ = node_->get_clock()->now();
    try {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      processAndDisplayImage(image);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "OpenCV exception: %s", e.what());
    }
  }


  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    last_image_received_time_ = node_->get_clock()->now();
    try {
      cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      processAndDisplayImage(image);
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "OpenCV exception while decoding compressed image: %s",
                   e.what());
    }
  }
  

  void processAndDisplayImage(const cv::Mat& image) {
    cv::Mat processed_image;
    if (roi_.width > 0 && roi_.height > 0 && roi_.x >= 0 && roi_.y >= 0 &&
        roi_.x + roi_.width <= image.cols && roi_.y + roi_.height <= image.rows) {
      processed_image = image(roi_);
    } else {
      processed_image = image;
    }

    QImage qimage(processed_image.data, processed_image.cols, processed_image.rows,
                  processed_image.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(qimage.rgbSwapped())
                         .scaled(this->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    setPixmap(pixmap);
  }  


  void process() {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = node_->get_clock()->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = node_->get_name();
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";

    double time_since_last_frame =
        (node_->get_clock()->now() - last_image_received_time_).seconds();
    if (time_since_last_frame > 1.0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "ERROR: Timeout image";
    }

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "Last Image Received";
    kv.value = std::to_string(last_image_received_time_.seconds());
    status.values.push_back(kv);
    
    diagnostic_msgs::msg::KeyValue kv_window_state;
    kv_window_state.key = "Image Shown";
    kv_window_state.value = isVisible() ? "True" : "False";
    status.values.push_back(kv_window_state);

    diag_array.status.push_back(status);
    diagnostic_pub_->publish(diag_array);
  }


  void visualizeTopicCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      show();
    } else {
      hide();
    }
  }


  void visualizeServiceCallback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
      show();
      response->success = true;
      response->message = "Visualize image.";
    } else {
      hide();
      response->success = true;
      response->message = "Hide image.";
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr process_timer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visualize_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr visualize_srv_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Time last_image_received_time_;

  std::string image_topic_;
  std::string compressed_image_topic_;
  std::string visualization_service_;
  std::string visualization_topic_;
  std::string position_;
  int screen_index_;
  int pos_x_;
  int pos_y_;
  int width_;
  int height_;
  bool stay_on_top_;
  bool show_startup_;
  bool use_compressed_;  
  cv::Rect roi_;
};


void runQtApp(int argc, char* argv[], rclcpp::Node::SharedPtr node) {
  QApplication app(argc, argv);
  QtWindow window(node);
  app.exec();
}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("image_view_qt");
  std::thread qt_thread([=]() { runQtApp(argc, argv, node); });
  rclcpp::spin(node);
  rclcpp::shutdown();
  qt_thread.join();
  return 0;
}
