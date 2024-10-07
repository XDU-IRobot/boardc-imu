
#include <memory>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include "serial/serial.h"

#include "decoder.h"

class BoardcImuNode : public rclcpp::Node {
 public:
  BoardcImuNode(rclcpp::NodeOptions options = rclcpp::NodeOptions()) : Node("boardc_imu_node", options) {
    RCLCPP_INFO(this->get_logger(), "Node startup");

    if (serial_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial port opened");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      return;
    }

    serial_rx_thread_ = std::thread([this] {
      while (rclcpp::ok()) {
        std::string a = serial_.read();
        decoder_ << a;
      }
    });
  }

  ~BoardcImuNode() {
    serial_.close();
    serial_rx_thread_.join();
  }

 private:
  std::thread serial_rx_thread_{};
  sensor_msgs::msg::Imu imu_msg_{};
  geometry_msgs::msg::QuaternionStamped quaternion_msg_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_{
      create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS())};
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_pub_{
      create_publisher<geometry_msgs::msg::QuaternionStamped>("quaternion", rclcpp::SensorDataQoS())};
  serial::Serial serial_{"/dev/boardc_imu", 921600, serial::Timeout::simpleTimeout(1000)};
  Decoder decoder_{[this](geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
    quaternion_msg_ = *msg;
    quaternion_pub_->publish(quaternion_msg_);
  }};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BoardcImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
