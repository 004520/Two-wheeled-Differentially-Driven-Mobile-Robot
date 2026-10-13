#include "mycar_servo_qt_gui/main_window.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QTime>
#include <QVBoxLayout>

namespace mycar_servo_qt_gui
{

MainWindow::MainWindow(std::shared_ptr<ServoPublisher> node, QWidget * parent)
: QMainWindow(parent), node_(std::move(node))
{
  setWindowTitle("MyCar 舵机调试");
  resize(620, 340);
  setMinimumSize(560, 300);

  auto * root = new QWidget(this);
  auto * layout = new QVBoxLayout(root);
  layout->setContentsMargins(14, 12, 14, 14);
  layout->setSpacing(12);
  layout->addWidget(createServoPanel());
  setCentralWidget(root);

  setStyleSheet(
    "QMainWindow,QWidget{background:#f6f8fb;color:#1f2937;font-family:'Noto Sans CJK SC','Microsoft YaHei',sans-serif;}"
    "QGroupBox{background:#ffffff;border:1px solid #e5eaf2;border-radius:10px;margin-top:10px;padding:8px;font-weight:700;color:#1f2937;}"
    "QGroupBox::title{subcontrol-origin:margin;left:12px;padding:0 6px;color:#1f2937;}"
    "QPushButton{background:#ffffff;border:1px solid #e5eaf2;border-radius:6px;padding:6px 12px;min-height:28px;color:#374151;font-weight:600;}"
    "QPushButton:hover{background:#f3f7ff;border-color:#b9d5ff;color:#1d6fe8;}"
    "QPushButton:pressed{background:#e6f0ff;}"
    "QPushButton#primaryButton{background:#2f80ed;color:white;border-color:#2f80ed;}"
    "QPushButton#primaryButton:hover{background:#1d6fe8;}"
    "QPushButton#stopButton{background:#ef4444;color:white;border-color:#ef4444;}"
    "QPushButton#stopButton:hover{background:#dc2626;}"
    "QDoubleSpinBox,QComboBox,QPlainTextEdit{background:#ffffff;border:1px solid #e5eaf2;border-radius:6px;padding:5px;color:#1f2937;}"
    "QLabel{background:transparent;color:#374151;}");
}

QWidget * MainWindow::createServoPanel()
{
  auto * group = new QGroupBox("舵机控制 /servo_cmd");
  auto * layout = new QGridLayout(group);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setHorizontalSpacing(10);
  layout->setVerticalSpacing(8);

  channel_combo_ = new QComboBox;
  for (int i = 1; i <= 4; ++i) {
    channel_combo_->addItem(QString("通道 %1").arg(i), i);
  }
  angle_spin_ = new QDoubleSpinBox;
  angle_spin_->setRange(-180.0, 180.0);
  angle_spin_->setDecimals(1);
  angle_spin_->setSingleStep(1.0);
  angle_spin_->setSuffix(" deg");
  angle_spin_->setValue(0.0);

  auto * send = makeButton("发送");
  auto * plus = makeButton("+");
  auto * minus = makeButton("-");
  auto * zero = makeButton("回正");
  auto * c1 = makeButton("1");
  auto * c2 = makeButton("2");
  auto * c3 = makeButton("3");
  auto * c4 = makeButton("4");

  send->setObjectName("primaryButton");
  zero->setObjectName("stopButton");

  connect(send, &QPushButton::clicked, this, &MainWindow::publishCurrent);
  connect(plus, &QPushButton::clicked, this, [this]() { stepAngle(1.0); });
  connect(minus, &QPushButton::clicked, this, [this]() { stepAngle(-1.0); });
  connect(zero, &QPushButton::clicked, this, &MainWindow::resetAngle);
  connect(c1, &QPushButton::clicked, this, [this]() { setChannel(1); });
  connect(c2, &QPushButton::clicked, this, [this]() { setChannel(2); });
  connect(c3, &QPushButton::clicked, this, [this]() { setChannel(3); });
  connect(c4, &QPushButton::clicked, this, [this]() { setChannel(4); });
  connect(channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
    publishCurrent();
  });
  connect(angle_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
    publishCurrent();
  });

  layout->addWidget(new QLabel("通道"), 0, 0);
  layout->addWidget(channel_combo_, 0, 1, 1, 3);
  layout->addWidget(new QLabel("角度"), 1, 0);
  layout->addWidget(angle_spin_, 1, 1, 1, 3);
  layout->addWidget(c1, 2, 0);
  layout->addWidget(c2, 2, 1);
  layout->addWidget(c3, 2, 2);
  layout->addWidget(c4, 2, 3);
  layout->addWidget(minus, 3, 0);
  layout->addWidget(plus, 3, 1);
  layout->addWidget(zero, 3, 2);
  layout->addWidget(send, 3, 3);

  current_label_ = new QLabel("当前: 通道 1, 角度 0.0 deg");
  status_label_ = new QLabel("待发送");
  log_view_ = new QPlainTextEdit;
  log_view_->setReadOnly(true);
  log_view_->setMaximumHeight(82);

  layout->addWidget(current_label_, 4, 0, 1, 4);
  layout->addWidget(status_label_, 5, 0, 1, 4);
  layout->addWidget(log_view_, 6, 0, 1, 4);

  publishCurrent();
  return group;
}

QPushButton * MainWindow::makeButton(const QString & text)
{
  auto * button = new QPushButton(text);
  button->setFocusPolicy(Qt::NoFocus);
  return button;
}

void MainWindow::publishCurrent()
{
  const int channel = channel_combo_->currentData().toInt();
  const double angle = angle_spin_->value();
  node_->publishCommand(channel, angle);
  current_label_->setText(QString("当前: 通道 %1, 角度 %2 deg").arg(channel).arg(angle, 0, 'f', 1));
  status_label_->setText(QString("已发送 /servo_cmd  x=%1  y=%2  z=0").arg(channel).arg(angle, 0, 'f', 1));
  appendLog(QString("%1 发送: x=%2 y=%3 z=0").arg(QTime::currentTime().toString("HH:mm:ss")).arg(channel).arg(angle, 0, 'f', 1));
}

void MainWindow::setChannel(int channel)
{
  const int index = channel_combo_->findData(channel);
  if (index >= 0) {
    channel_combo_->setCurrentIndex(index);
  }
}

void MainWindow::stepAngle(double delta)
{
  angle_spin_->setValue(angle_spin_->value() + delta);
}

void MainWindow::resetAngle()
{
  angle_spin_->setValue(0.0);
}

void MainWindow::appendLog(const QString & text)
{
  log_view_->appendPlainText(text);
}

}  // namespace mycar_servo_qt_gui
