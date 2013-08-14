#include "RealplotWrapper.h"

#include "RealWidget.h"
#include "graphics/qt/common/Events.h"
#include <qapplication.h>
#include <QThread>


RealplotWrapper::RealplotWrapper()
    :widget_(NULL)
{
  if(QCoreApplication::instance() == NULL)
    return; //TODO: throw exception here in Iris
  if(QCoreApplication::instance()->thread() == QThread::currentThread())
  {
    connect( this, SIGNAL( createWidgetSignal() ), this, SLOT(createWidgetSlot()) );
    connect( this, SIGNAL( destroyWidgetSignal() ), this, SLOT(destroyWidgetSlot()) );
  }
  else
  {
    connect( this, SIGNAL( createWidgetSignal() ), this, SLOT(createWidgetSlot()), Qt::BlockingQueuedConnection );
    connect( this, SIGNAL( destroyWidgetSignal() ), this, SLOT(destroyWidgetSlot()), Qt::BlockingQueuedConnection );
    moveToThread(QCoreApplication::instance()->thread());
  }
  emit createWidgetSignal();
}

RealplotWrapper::~RealplotWrapper()
{
  emit destroyWidgetSignal();
}

void RealplotWrapper::createWidgetSlot()
{
  widget_ = new RealWidget;
  connect(this, SIGNAL(setWidgetTitle(QString)),
          widget_, SLOT(setWidgetTitle(QString)));
  connect(this, SIGNAL(setWidgetAxisLabels(QString, QString)),
          widget_, SLOT(setWidgetAxisLabels(QString, QString)));
  connect(this, SIGNAL(setWidgetXAxisScale(double,double)),
          widget_, SLOT(setWidgetXAxisScale(double,double)));
  connect(this, SIGNAL(setWidgetYAxisScale(double,double)),
          widget_, SLOT(setWidgetYAxisScale(double,double)));
  connect(this, SIGNAL(setWidgetXAxisAutoScale(bool)),
          widget_, SLOT(setWidgetXAxisAutoScale(bool)));
  connect(this, SIGNAL(setWidgetYAxisAutoScale(bool)),
          widget_, SLOT(setWidgetYAxisAutoScale(bool)));
  connect(this, SIGNAL(setWidgetXAxisRange(double,double)),
          widget_, SLOT(setWidgetXAxisRange(double,double)));

  widget_->resize( 800, 600 );
  widget_->show();
}

void RealplotWrapper::destroyWidgetSlot()
{
  delete widget_;
}

void RealplotWrapper::setNewData(double* data, int numPoints)
{
  if(widget_ == NULL)
    return; //TODO: throw exception here in Iris
  qApp->postEvent(widget_, new RealDataEvent(data, numPoints));
}

void RealplotWrapper::setNewData(float* data, int numPoints)
{
  if(widget_ == NULL)
    return; //TODO: throw exception here in Iris
  qApp->postEvent(widget_, new RealDataEvent(data, numPoints));
}

void RealplotWrapper::setTitle(std::string title)
{
  if(widget_ == NULL)
    return;
  QString str = QString::fromUtf8(title.c_str());
  emit setWidgetTitle(str);
}

void RealplotWrapper::setAxisLabels(std::string xLabel, std::string yLabel)
{
  if(widget_ == NULL)
    return;
  QString xStr = QString::fromUtf8(xLabel.c_str());
  QString yStr = QString::fromUtf8(yLabel.c_str());
  emit setWidgetAxisLabels(xStr, yStr);
}

void RealplotWrapper::setXAxisScale(double xMin, double xMax)
{
  if(widget_ == NULL)
    return;
  emit setWidgetXAxisScale(xMin, xMax);
}

void RealplotWrapper::setYAxisScale(double yMin, double yMax)
{
  if(widget_ == NULL)
    return;
  emit setWidgetYAxisScale(yMin, yMax);
}

void RealplotWrapper::setXAxisAutoScale(bool on=true)
{
  if(widget_ == NULL)
    return;
  emit setWidgetXAxisAutoScale(on);
}

void RealplotWrapper::setYAxisAutoScale(bool on=true)
{
  if(widget_ == NULL)
    return;
  emit setWidgetYAxisAutoScale(on);
}

void RealplotWrapper::setXAxisRange(double xMin, double xMax)
{
  if(widget_ == NULL)
    return;
  emit setWidgetXAxisRange(xMin, xMax);
}
