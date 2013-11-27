#include "VectorRPYGraphicWidget.h"

// system includes

// library includes
#include <QPainter>
#include <QDoubleValidator>

// custom includes
#include "PaintPrimitives.h"


/*---------------------------------- public: -----------------------------{{{-*/
VectorRPYGraphicWidget::VectorRPYGraphicWidget(QWidget* p_parent)
  :QWidget(p_parent)
{
  setMinimumWidth(300);
  setFixedHeight(135);

  createChildWidgets();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- protected: ----------------------------{{{-*/
void
VectorRPYGraphicWidget::paintEvent(QPaintEvent* p_event)
{
  QPainter painter(this);
  int editWidth = 60;
  int editHeight = 25;
  int arrowLineWidth = 3;
  int rotationDiameter = 25;
  int border = rotationDiameter / 2;
  int editDistance = 5;
  QPoint axisOrigin(border + editWidth, height() - (border + editHeight/2));
  std::vector<QPoint> axisTips = PaintPrimitives::drawCoordinateAxis(painter, axisOrigin, arrowLineWidth, true, rotationDiameter);
  QPoint& xAxisTip = axisTips[0];
  QPoint& yAxisTip = axisTips[1];
  QPoint& zAxisTip = axisTips[2];
  QPoint& xAxisRotationCenter = axisTips[3];
  QPoint& yAxisRotationCenter = axisTips[4];
  QPoint& zAxisRotationCenter = axisTips[5];

  m_xEdit->setGeometry(xAxisTip.x() + editDistance, xAxisTip.y() - editHeight/2, editWidth, editHeight);
  m_rxEdit->setGeometry(xAxisRotationCenter.x() - (rotationDiameter/4 + editWidth), height() - editHeight, editWidth, editHeight);

  m_yEdit->setGeometry(yAxisTip.x() + (editDistance - editWidth/4), yAxisTip.y() - (editDistance + editHeight), editWidth, editHeight);
  m_ryEdit->setGeometry(yAxisRotationCenter.x() + rotationDiameter/2, yAxisRotationCenter.y() - rotationDiameter/4, editWidth, editHeight);

  m_zEdit->setGeometry(zAxisTip.x() - editWidth/2, zAxisTip.y() - (editDistance + editHeight), editWidth, editHeight);
  m_rzEdit->setGeometry(0, zAxisRotationCenter.y() - editHeight/2, editWidth, editHeight);

}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
VectorRPYGraphicWidget::createChildWidgets()
{
  m_xEdit = new QLineEdit(this);
  m_yEdit = new QLineEdit(this);
  m_zEdit = new QLineEdit(this);

  m_rxEdit = new QLineEdit(this);
  m_ryEdit = new QLineEdit(this);
  m_rzEdit = new QLineEdit(this);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
