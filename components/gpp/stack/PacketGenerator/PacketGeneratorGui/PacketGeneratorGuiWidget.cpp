#include "PacketGeneratorGuiWidget.h"

#include <QLayout>
#include <QLabel>
#include <QSpinBox>
#include <QShortcut>
#include <QPushButton>

PacketGeneratorGuiWidget::PacketGeneratorGuiWidget(PGGuiCallback* callback,
                                         QWidget *parent)
  :QWidget(parent)
{
  callback_ = callback;

  // Create widgets here
  QFont font("Helvetica", 10);
  font.setBold(true);
  label_ = new QLabel("Packet Length", this);
  label_->setFont(font);
  label_->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

  lengthBox_ = new QSpinBox(this);
  lengthBox_->setRange(1,2048);
  lengthBox_->setSingleStep(1);
  lengthBox_->setValue(1024);

  b_ = new QPushButton("Generate");
  connect(b_, SIGNAL(clicked()), this, SLOT(generatePacket()));

  QHBoxLayout* hLayout1 = new QHBoxLayout();
  hLayout1->addWidget(label_);
  hLayout1->addWidget(lengthBox_);

  QVBoxLayout* vLayout1 = new QVBoxLayout(this);
  vLayout1->addLayout(hLayout1);
  vLayout1->addWidget(b_);

  setPalette( QPalette( QColor( 192, 192, 192 ) ) );
  updateGradient();
}

PacketGeneratorGuiWidget::~PacketGeneratorGuiWidget()
{
}

void PacketGeneratorGuiWidget::generatePacket()
{
  int l = lengthBox_->value();
  callback_->generatePacket(l);
}

void PacketGeneratorGuiWidget::updateGradient()
{
    QPalette pal = palette();

    const QColor buttonColor = pal.color( QPalette::Button );
    const QColor lightColor = pal.color( QPalette::Light );
    const QColor midLightColor = pal.color( QPalette::Midlight );

#ifdef Q_WS_X11
    // Qt 4.7.1: QGradient::StretchToDeviceMode is buggy on X11

    QLinearGradient gradient( rect().topLeft(), rect().topRight() );
#else
    QLinearGradient gradient( 0, 0, 1, 0 );
    gradient.setCoordinateMode( QGradient::StretchToDeviceMode );
#endif

    gradient.setColorAt( 0.0, midLightColor );
    gradient.setColorAt( 0.7, buttonColor );
    gradient.setColorAt( 1.0, buttonColor );

    pal.setBrush( QPalette::Window, gradient );
    setPalette( pal );
}
