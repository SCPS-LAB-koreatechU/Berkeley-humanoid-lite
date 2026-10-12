// ZED 2i mono (left) camera publisher straight from UVC (no ZED SDK, no hobot_zed_cam).
// The ZED streams left|right side by side in YUYV; this crops the left half and publishes it
// as bgr8 (rqt/cv_bridge friendly) or camera-native yuv422_yuy2 (no color conversion).
// Written in C++ because rclpy needs ~70 ms to publish one 1280x720 bgr8 image on the X5.

#include <atomic>
#include <map>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ZedMonoNode : public rclcpp::Node
{
public:
  ZedMonoNode() : Node("zed_mono_node")
  {
    // side-by-side stream size per ZED mode
    static const std::map<std::string, cv::Size> modes = {
      {"vga", {1344, 376}}, {"hd720", {2560, 720}}, {"hd1080", {3840, 1080}}, {"hd2k", {4416, 1242}}};
    auto device = declare_parameter<std::string>("device", "/dev/video0");
    auto mode = declare_parameter<std::string>("resolution", "hd720");
    auto fps = declare_parameter<int>("fps", 30);
    frame_id_ = declare_parameter<std::string>("frame_id", "zed_left_camera");
    encoding_ = declare_parameter<std::string>("encoding", "bgr8");
    if (!modes.count(mode)) throw std::runtime_error("unknown resolution: " + mode);
    if (encoding_ != "bgr8" && encoding_ != "yuv422_yuy2") throw std::runtime_error("encoding must be bgr8 or yuv422_yuy2");

    if (!cap_.open(device, cv::CAP_V4L2)) throw std::runtime_error("cannot open " + device);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, modes.at(mode).width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, modes.at(mode).height);
    cap_.set(cv::CAP_PROP_FPS, fps);
    // >1 V4L2 buffers: with a single buffer every frame arriving during processing is dropped
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 4);
    cap_.set(cv::CAP_PROP_CONVERT_RGB, 0);  // raw YUYV; convert only the left half ourselves
    width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)) / 2;
    height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

    pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rclcpp::QoS(rclcpp::KeepLast(5)));
    RCLCPP_INFO(get_logger(), "%s: %dx%d@%.0f -> left %dx%d %s on %s", device.c_str(), width_ * 2, height_,
                cap_.get(cv::CAP_PROP_FPS), width_, height_, encoding_.c_str(), pub_->get_topic_name());
    thread_ = std::thread(&ZedMonoNode::loop, this);
  }

  ~ZedMonoNode() override
  {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    cap_.release();
  }

private:
  void loop()
  {
    cv::Mat raw;
    int count = 0;
    auto t0 = now();
    while (!stop_ && rclcpp::ok()) {
      if (!cap_.read(raw) || raw.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "frame grab failed (try: ros2 run zed_imu_ros2 zed_reset)");
        continue;
      }
      // raw YUYV arrives as a flat byte buffer: view as (h, 2w*2 bytes) and keep the left half
      cv::Mat yuyv = raw.reshape(1, height_).colRange(0, width_ * 2);

      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = now();
      msg->header.frame_id = frame_id_;
      msg->height = height_;
      msg->width = width_;
      msg->encoding = encoding_;
      if (encoding_ == "bgr8") {
        msg->step = width_ * 3;
        msg->data.resize(msg->step * height_);
        cv::Mat bgr(height_, width_, CV_8UC3, msg->data.data());
        cv::cvtColor(yuyv.clone().reshape(2, height_), bgr, cv::COLOR_YUV2BGR_YUYV);
      } else {
        msg->step = width_ * 2;
        msg->data.resize(msg->step * height_);
        yuyv.copyTo(cv::Mat(height_, width_ * 2, CV_8UC1, msg->data.data()));
      }
      pub_->publish(std::move(msg));

      if (++count, (now() - t0).seconds() >= 5.0) {
        RCLCPP_INFO(get_logger(), "publish rate %.1f Hz", count / (now() - t0).seconds());
        count = 0;
        t0 = now();
      }
    }
  }

  cv::VideoCapture cap_;
  int width_, height_;
  std::string frame_id_, encoding_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ZedMonoNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("zed_mono_node"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
