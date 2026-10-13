#include "mycar_servo_qt_gui/main_window.hpp"

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  setenv("QT_QPA_PLATFORM", "xcb", 0);
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<mycar_servo_qt_gui::ServoPublisher>();
  auto * window = new mycar_servo_qt_gui::MainWindow(node);
  window->show();

  const int result = app.exec();
  rclcpp::shutdown();
  return result;
}
