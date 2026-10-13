#pragma once

#include <QImage>
#include <QWidget>

namespace mycar_qt_gui
{

class CameraView : public QWidget
{
  Q_OBJECT

public:
  explicit CameraView(QWidget * parent = nullptr);
  void setImage(const QImage & image);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QImage image_;
};

}  // namespace mycar_qt_gui
