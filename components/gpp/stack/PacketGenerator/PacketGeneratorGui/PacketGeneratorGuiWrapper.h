#ifndef PACKETGENERATORGUIWRAPPER_H
#define PACKETGENERATORGUIWRAPPER_H

#include <qapplication.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "PGGuiCallback.h"

class PacketGeneratorGuiWidget;

class PacketGeneratorGuiWrapper
    : QObject, public PGGuiCallback
{
  Q_OBJECT

public:
  PacketGeneratorGuiWrapper();
  ~PacketGeneratorGuiWrapper();
  void waitForPacket(int& length);

  virtual void generatePacket(int length);

public slots:
  void createWidgetSlot();
  void destroyWidgetSlot();
  void widgetDestroyed();

signals:
  void createWidgetSignal();
  void destroyWidgetSignal();
  void destroyWidgetSignalBlocking();

private:
  PacketGeneratorGuiWidget* widget_;
  bool destroyed_;
  boost::mutex mutex_;
  boost::condition_variable cond_;
  int pLength_;
};

#endif // PACKETGENERATORGUIWRAPPER_H
