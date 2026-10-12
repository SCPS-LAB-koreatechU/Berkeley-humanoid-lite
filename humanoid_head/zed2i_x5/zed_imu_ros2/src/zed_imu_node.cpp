// ZED 2i IMU publisher for ROS 2, built on zed-open-capture (no ZED SDK / CUDA).
// Publishes (relative to node namespace):
//   imu/data_raw      sensor_msgs/Imu            ~400 Hz (accel + gyro, no orientation)
//   imu/mag           sensor_msgs/MagneticField  ~50 Hz  (raw, uncalibrated)
//   atm_press         sensor_msgs/FluidPressure  ~25 Hz
//   temperature/imu   sensor_msgs/Temperature    ~25 Hz
//   humidity          sensor_msgs/RelativeHumidity ~25 Hz
//   temperature/left, temperature/right  sensor_msgs/Temperature (CMOS sensors)

#include <atomic>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensorcapture.hpp"

namespace oc = sl_oc::sensors;

static constexpr double kDeg2Rad = M_PI / 180.0;
// Noise measured on serial 39153075 at rest (2026-10-12): accel std ~0.009 m/s^2, gyro std ~0.08 deg/s
static constexpr double kAccelVar = 1e-4;
static constexpr double kGyroVar = 2e-6;

class ZedImuNode : public rclcpp::Node
{
public:
  ZedImuNode() : Node("zed_imu_node")
  {
    serial_ = declare_parameter<int>("serial_number", -1);
    frame_id_ = declare_parameter<std::string>("frame_id", "zed_imu_link");
    publish_aux_ = declare_parameter<bool>("publish_aux", true);
    calib_sec_ = declare_parameter<double>("gyro_bias_calib_sec", 0.0);

    sens_ = std::make_unique<oc::SensorCapture>(sl_oc::VERBOSITY::ERROR);
    if (!sens_->initializeSensors(serial_)) {
      throw std::runtime_error("ZED sensor module not found (check USB / udev rules)");
    }
    uint16_t fw_maj, fw_min;
    sens_->getFirmwareVersion(fw_maj, fw_min);
    RCLCPP_INFO(get_logger(), "Connected to ZED sn %d, firmware %u.%u",
                sens_->getSerialNumber(), fw_maj, fw_min);

    // Reliable by default: compatible with both reliable (rqt, ros2 topic echo) and best-effort
    // subscribers. Best-effort publishers are invisible to reliable subscribers.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    if (!declare_parameter<bool>("reliable_qos", true)) qos.best_effort();
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", qos);
    if (publish_aux_) {
      mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
      press_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("atm_press", qos);
      temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("temperature/imu", qos);
      hum_pub_ = create_publisher<sensor_msgs::msg::RelativeHumidity>("humidity", qos);
      tl_pub_ = create_publisher<sensor_msgs::msg::Temperature>("temperature/left", qos);
      tr_pub_ = create_publisher<sensor_msgs::msg::Temperature>("temperature/right", qos);
    }

    if (calib_sec_ > 0.0) calibrateGyro();

    imu_thread_ = std::thread(&ZedImuNode::imuLoop, this);
    if (publish_aux_) aux_thread_ = std::thread(&ZedImuNode::auxLoop, this);
  }

  ~ZedImuNode() override
  {
    stop_ = true;
    if (imu_thread_.joinable()) imu_thread_.join();
    if (aux_thread_.joinable()) aux_thread_.join();
  }

private:
  // Device timestamps (serial 39153075) were ~130 s behind the host and drifted ~7 ms/s,
  // so samples are stamped with the host clock on receipt (HID latency is ~1 ms).
  rclcpp::Time stamp(uint64_t /*dev_ns*/) { return this->now(); }

  void calibrateGyro()
  {
    RCLCPP_INFO(get_logger(), "Calibrating gyro bias for %.1f s, keep the camera still...", calib_sec_);
    double sx = 0, sy = 0, sz = 0;
    int n = 0;
    auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(calib_sec_);
    while (std::chrono::steady_clock::now() < end) {
      oc::data::Imu d = sens_->getLastIMUData(5000);
      if (d.valid != oc::data::Imu::NEW_VAL) continue;
      sx += d.gX; sy += d.gY; sz += d.gZ; n++;
    }
    if (n > 0) {
      bias_[0] = sx / n; bias_[1] = sy / n; bias_[2] = sz / n;
    }
    RCLCPP_INFO(get_logger(), "Gyro bias [deg/s] (%d samples): %.4f %.4f %.4f", n, bias_[0], bias_[1], bias_[2]);
  }

  void imuLoop()
  {
    while (!stop_ && rclcpp::ok()) {
      oc::data::Imu d = sens_->getLastIMUData(5000);
      if (d.valid != oc::data::Imu::NEW_VAL) continue;

      sensor_msgs::msg::Imu msg;
      msg.header.stamp = stamp(d.timestamp);
      msg.header.frame_id = frame_id_;
      msg.orientation_covariance[0] = -1.0;  // no orientation estimate
      msg.linear_acceleration.x = d.aX;
      msg.linear_acceleration.y = d.aY;
      msg.linear_acceleration.z = d.aZ;
      msg.angular_velocity.x = (d.gX - bias_[0]) * kDeg2Rad;
      msg.angular_velocity.y = (d.gY - bias_[1]) * kDeg2Rad;
      msg.angular_velocity.z = (d.gZ - bias_[2]) * kDeg2Rad;
      for (int i = 0; i < 3; i++) {
        msg.linear_acceleration_covariance[i * 4] = kAccelVar;
        msg.angular_velocity_covariance[i * 4] = kGyroVar;
      }
      imu_pub_->publish(msg);
    }
  }

  void auxLoop()
  {
    // The library flips `valid` to OLD_VAL on every following HID packet without changing the
    // timestamp, so a new sample is detected by its timestamp instead of NEW_VAL.
    uint64_t last_mag_ts = 0, last_env_ts = 0, last_cam_ts = 0;
    while (!stop_ && rclcpp::ok()) {
      oc::data::Magnetometer m = sens_->getLastMagnetometerData(100);
      if (m.valid != oc::data::Magnetometer::NOT_PRESENT && m.timestamp != last_mag_ts) {
        last_mag_ts = m.timestamp;
        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp = stamp(m.timestamp);
        msg.header.frame_id = frame_id_;
        msg.magnetic_field.x = m.mX * 1e-6;  // uT -> T
        msg.magnetic_field.y = m.mY * 1e-6;
        msg.magnetic_field.z = m.mZ * 1e-6;
        mag_pub_->publish(msg);
      }

      oc::data::Environment e = sens_->getLastEnvironmentData(100);
      if (e.valid != oc::data::Environment::NOT_PRESENT && e.timestamp != last_env_ts &&
          e.press > 0.0f) {  // first packet is garbage
        last_env_ts = e.timestamp;
        auto st = stamp(e.timestamp);
        sensor_msgs::msg::FluidPressure p;
        p.header.stamp = st;
        p.header.frame_id = frame_id_;
        p.fluid_pressure = e.press * 100.0;  // hPa -> Pa
        press_pub_->publish(p);

        sensor_msgs::msg::Temperature t;
        t.header = p.header;
        t.temperature = e.temp;
        temp_pub_->publish(t);

        sensor_msgs::msg::RelativeHumidity h;
        h.header = p.header;
        h.relative_humidity = e.humid / 100.0;  // %rH -> 0..1
        hum_pub_->publish(h);
      }

      oc::data::Temperature c = sens_->getLastCameraTemperatureData(100);
      if (c.valid != oc::data::Temperature::NOT_PRESENT && c.timestamp != last_cam_ts &&
          c.temp_left > -100.0f) {
        last_cam_ts = c.timestamp;
        sensor_msgs::msg::Temperature t;
        t.header.stamp = stamp(c.timestamp);
        t.header.frame_id = frame_id_;
        t.temperature = c.temp_left;
        tl_pub_->publish(t);
        t.temperature = c.temp_right;
        tr_pub_->publish(t);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::unique_ptr<oc::SensorCapture> sens_;
  int serial_;
  std::string frame_id_;
  bool publish_aux_;
  double calib_sec_;
  double bias_[3] = {0.0, 0.0, 0.0};
  std::atomic<bool> stop_{false};
  std::thread imu_thread_, aux_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr press_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_, tl_pub_, tr_pub_;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr hum_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ZedImuNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("zed_imu_node"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
