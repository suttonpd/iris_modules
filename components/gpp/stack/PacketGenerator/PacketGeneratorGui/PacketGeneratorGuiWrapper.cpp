#include "PacketGeneratorGuiWrapper.h"

#include "PacketGeneratorGuiWidget.h"
#include "graphics/qt/common/Events.h"
#include <qapplication.h>
#include <QThread>

using namespace std;

PacketGeneratorGuiWrapper::PacketGeneratorGuiWrapper()
    :widget_(NULL)
    ,destroyed_(true)
{
  if(QCoreApplication::instance() == NULL)
    return; //TODO: throw exception here in Iris
  if(QCoreApplication::instance()->thread() == QThread::currentThread())
  {
    connect( this, SIGNAL( createWidgetSignal() ),
             this, SLOT(createWidgetSlot()) );
    connect( this, SIGNAL( destroyWidgetSignal() ),
             this, SLOT(destroyWidgetSlot()) );
    connect( this, SIGNAL( destroyWidgetSignalBlocking() ),
             this, SLOT(destroyWidgetSlot()) );
  }
  else
  {
    connect( this, SIGNAL( createWidgetSignal() ),
             this, SLOT(createWidgetSlot()),
             Qt::BlockingQueuedConnection );
    connect( this, SIGNAL( destroyWidgetSignal() ),
             this, SLOT(destroyWidgetSlot()) );
    connect( this, SIGNAL( destroyWidgetSignalBlocking() ),
             this, SLOT(destroyWidgetSlot()),
             Qt::BlockingQueuedConnection );
    moveToThread(QCoreApplication::instance()->thread());
  }
  emit createWidgetSignal();
}

PacketGeneratorGuiWrapper::~PacketGeneratorGuiWrapper()
{
  if(destroyed_)
    emit destroyWidgetSignal();
  else
    emit destroyWidgetSignalBlocking();
}

void PacketGeneratorGuiWrapper::waitForPacket(int& length)
{
  boost::mutex::scoped_lock lock(mutex_);
  cond_.wait(lock);
  length = pLength_;
}

void PacketGeneratorGuiWrapper::generatePacket(int length)
{
  boost::mutex::scoped_lock lock(mutex_);
  pLength_ = length;
  lock.unlock();
  cond_.notify_one();
}

void PacketGeneratorGuiWrapper::createWidgetSlot()
{
  widget_ = new PacketGeneratorGuiWidget(this);
  destroyed_ = false;
  widget_->setAttribute(Qt::WA_DeleteOnClose, true);
  connect(widget_, SIGNAL( destroyed() ),
          this, SLOT( widgetDestroyed() ));
  widget_->show();
}

void PacketGeneratorGuiWrapper::destroyWidgetSlot()
{
  if(widget_)
    delete widget_;
  widget_ = NULL;
}

void PacketGeneratorGuiWrapper::widgetDestroyed()
{
  destroyed_ = true;
}
