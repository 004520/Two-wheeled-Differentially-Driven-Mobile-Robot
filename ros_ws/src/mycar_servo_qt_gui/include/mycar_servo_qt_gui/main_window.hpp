#pragma once

#include "mycar_servo_qt_gui/servo_publisher.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>

#include <memory>

namespace mycar_servo_qt_gui
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(std::shared_ptr<ServoPublisher> node, QWidget * parent = nullptr);

private:
  QWidget * createServoPanel();
  QPushButton * makeButton(const QString & text);
  void publishCurrent();
  void setChannel(int channel);
  void stepAngle(double delta);
  void resetAngle();
  void appendLog(const QString & text);

  std::shared_ptr<ServoPublisher> node_;
  QComboBox * channel_combo_{nullptr};
  QDoubleSpinBox * angle_spin_{nullptr};
  QLabel * current_label_{nullptr};
  QLabel * status_label_{nullptr};
  QPlainTextEdit * log_view_{nullptr};
};

}  // namespace mycar_servo_qt_gui
