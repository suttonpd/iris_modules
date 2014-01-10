#ifndef PACKETGENERATORGUIWIDGET_H
#define PACKETGENERATORGUIWIDGET_H

#include <qapplication.h>
#include <qwidget.h>

#include "PGGuiCallback.h"

class QSpinBox;
class QPushButton;
class QLabel;

class PacketGeneratorGuiWidget
  : public QWidget
{
  Q_OBJECT

public:
  PacketGeneratorGuiWidget(PGGuiCallback* callback = NULL,
                      QWidget* parent = NULL);
  virtual ~PacketGeneratorGuiWidget();

public slots:
  void generatePacket();

protected:

private:
  void updateGradient();
  PGGuiCallback* callback_;
  QLabel* label_;
  QSpinBox* lengthBox_;
  QPushButton* b_;
};

#endif // PACKETGENERATORGUIWIDGET_H
