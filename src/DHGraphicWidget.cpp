#include "DHGraphicWidget.h"

// system includes

// library includes
#include <QPainter>
#include <QDoubleValidator>

// custom includes
#include "PaintPrimitives.h"


/*---------------------------------- public: -----------------------------{{{-*/
DHGraphicWidget::DHGraphicWidget(QWidget* p_parent)
  :QWidget(p_parent)
{
  setMinimumWidth(350);
  setFixedHeight(213);

  createChildWidgets();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- protected: ----------------------------{{{-*/
void
DHGraphicWidget::paintEvent(QPaintEvent* p_event)
{
  QPainter painter(this);
  painter.drawImage(m_dhImage->rect(), *m_dhImage, m_dhImage->rect());
  int editWidth = 60;
  int editHeight = 25;

  m_dEdit->setGeometry(10, 165, editWidth, editHeight);
  m_aEdit->setGeometry(182, 145, editWidth, editHeight);
  m_thetaEdit->setGeometry(10, 58, editWidth, editHeight);
  m_alphaEdit->setGeometry(155, 110, editWidth, editHeight);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
DHGraphicWidget::createChildWidgets()
{
  m_dEdit = new QLineEdit(this);
  m_aEdit = new QLineEdit(this);
  m_thetaEdit = new QLineEdit(this);
  m_alphaEdit = new QLineEdit(this);

  m_dhImage = new QImage(QString(":/img/dh.png"));
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
