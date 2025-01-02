
# **image_view_qt**

The **image_view_qt** is a ROS based graphical application for visualizing raw or compressed images in a frameless QT window. It is designed to be highly configurable, enabling users to adjust display settings, crop regions of interest (ROI), control window positioning, and toggle visualization dynamically.

---

## **Features**

- **Image Visualization**:
  - Displays images from raw (`sensor_msgs/Image`) or compressed (`sensor_msgs/CompressedImage`) topics.
  - Automatic decoding of compressed images.
  - Support for ROI (Region of Interest) cropping.

- **Configurable Display**:
  - Adjustable window size, position, and alignment on the screen.
  - Multi-screen support with `screen_index` selection.
  - Options to set the window to stay on top or show on startup.

- **Dynamic Visualization Control**:
  - Enable/disable visualization dynamically using a ROS 2 topic or service.

- **Diagnostics**:
  - Publishes diagnostic information to monitor image reception status, including whether the window is currently visible.

---

## **Dependencies**

Ensure the following dependencies are installed:

- **ROS 2** (tested with Humble/Foxy)
- **OpenCV** (with `imgcodecs` module enabled)
- **QT Framework**

---

## **Installation**

1. Clone the repository into your ROS 2 workspace:
   ```bash
   git clone https://github.com/markusschratter/image_view_qt ~/ros2_ws/src/image_view_qt
   ```

2. Install dependencies:
   ```bash
   sudo apt install ros-humble-cv-bridge qt5-default libopencv-dev
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select image_view_qt
   ```


---

## **Configuration**

The node is configured via ROS 2 parameters. Below are the available parameters and their descriptions:

### **Parameters**

| Parameter                | Type       | Default Value                       | Description                                                                                   |
|--------------------------|------------|-------------------------------------|-----------------------------------------------------------------------------------------------|
| `show_startup`           | `bool`     | `false`                             | Whether to show the window on startup.                                                       |
| `stay_on_top`            | `bool`     | `false`                             | If `true`, the window will stay on top of all other windows.                                  |
| `use_compressed`         | `bool`     | `false`                             | Use compressed image topics (`sensor_msgs/CompressedImage`) if `true`.                       |
| `image_topic`            | `string`   | `/image_raw`   | The topic to subscribe to for raw images.                                                    |
| `compressed_image_topic` | `string`   | `/image_raw/compressed`  | The topic to subscribe to for compressed images.                                             |
| `visualization_service`  | `string`   | `/visualization`                    | The ROS 2 service for toggling the visualization.                                            |
| `visualization_topic`    | `string`   | `/visualization`                    | The ROS 2 topic for toggling the visualization.                                              |
| `pos_x`                  | `int`      | `0`                                 | Custom x-coordinate for the window position.                                                 |
| `pos_y`                  | `int`      | `0`                                 | Custom y-coordinate for the window position.                                                 |
| `width`                  | `int`      | `800`                               | The width of the QT window in pixels.                                                        |
| `height`                 | `int`      | `600`                               | The height of the QT window in pixels.                                                       |
| `roi_x`                  | `int`      | `0`                                 | The x-coordinate of the ROI (Region of Interest).                                             |
| `roi_y`                  | `int`      | `0`                                 | The y-coordinate of the ROI.                                                                 |
| `roi_width`              | `int`      | `-1`                                | The width of the ROI. Use `-1` for full image width.                                          |
| `roi_height`             | `int`      | `-1`                                | The height of the ROI. Use `-1` for full image height.                                        |
| `position`               | `string`   | `top:center`                        | Predefined positions: `top:left`, `top:center`, `top:right`, `middle:left`, `middle:center`, `middle:right`, `bottom:left`, `bottom:center`, `bottom:right`. |
| `screen_index`           | `int`      | `0`                                 | The screen index to display the window on (multi-screen setups).                              |

---

### **Example Configuration**

You can configure the node using a YAML file. Below is an example configuration:

```yaml
/image_view_qt_node:
  ros__parameters:
    show_startup: true
    stay_on_top: false
    use_compressed: true
    image_topic: "/image_raw"
    compressed_image_topic: "/image_raw/compressed"
    visualization_service: "/visualization"
    visualization_topic: "/visualization"
    pos_x: 0
    pos_y: 0
    width: 800
    height: 600
    roi_x: 0
    roi_y: 0
    roi_width: -1
    roi_height: -1
    position: "top:center"
    screen_index: 1
```

---

## **Running the Node**

1. Start the node:
   ```bash
   ros2 run image_view_qt image_view_qt_node --ros-args --params-file <path_to_yaml_file>
   ```

2. Use the following commands to dynamically control the visualization:

   - **Service**:
     ```bash
     ros2 service call /visualization std_srvs/srv/SetBool "{data: true}"
     ```
   - **Topic**:
     ```bash
     ros2 topic pub /visualization std_msgs/msg/Bool "{data: true}" -r 1
     ```

---

## **Diagnostics**

The node publishes diagnostic information on the `/diagnostics` topic, which includes:
- **Last Image Received**: Timestamp of the last image processed.
- **Timeout Detection**: Reports an error if no image is received for more than 1 second.
- **Image Shown**: Indicates whether the image window is currently visible.

---

## **Features in Action**

- **Multi-Screen Support**:
  - Use `screen_index` to specify the monitor for displaying the window.

- **Custom Positions**:
  - Set predefined positions using the `position` parameter (e.g., `top:left`, `bottom:right`).
  - Configure custom positions using `pos_x` and `pos_y`.

- **Dynamic Visualization**:
  - Toggle visualization on/off via ROS 2 services or topics.

---

## **Known Limitations**

- **Compressed Images**:
  - Ensure the topic provides valid compressed image data.
- **Multi-Screen Setup**:
  - Verify the `screen_index` matches your monitor setup.

---

## **Contributing**

We welcome contributions! Feel free to:
- Report issues
- Submit pull requests
- Suggest enhancements

---

## **License**

This project is licensed under the [Apache 2.0](LICENSE).

---

## **Acknowledgments**

Special thanks to the ROS and OpenCV communities for their incredible support and resources.

---

Enjoy seamless image visualization with the **Image View QT Node**! 🚀