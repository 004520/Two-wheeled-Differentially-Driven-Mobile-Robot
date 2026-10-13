#pragma once

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace mycar_servo_qt_gui
{

class ServoPublisher : public rclcpp::Node
{
public:
  ServoPublisher();
  void publishCommand(int channel, double angle_deg);

private:
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr servo_pub_;
};

}  // namespace mycar_servo_qt_gui
