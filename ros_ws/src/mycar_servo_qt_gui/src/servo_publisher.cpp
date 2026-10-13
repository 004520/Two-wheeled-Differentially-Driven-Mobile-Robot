#include "mycar_servo_qt_gui/servo_publisher.hpp"

namespace mycar_servo_qt_gui
{

ServoPublisher::ServoPublisher()
: Node("mycar_servo_qt_gui")
{
  declare_parameter("servo_topic", "/servo_cmd");
  const auto servo_topic = get_parameter("servo_topic").as_string();
  servo_pub_ = create_publisher<geometry_msgs::msg::Vector3>(servo_topic, 10);
}

void ServoPublisher::publishCommand(int channel, double angle_deg)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = static_cast<double>(channel);
  msg.y = angle_deg;
  msg.z = 0.0;
  servo_pub_->publish(msg);
}

}  // namespace mycar_servo_qt_gui
