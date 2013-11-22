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
  QColor xAxisColor("red");
  QColor yAxisColor("green");
  QColor zAxisColor("blue");
  int arrowLineWidth = 3;
  int rotationDiameter = 25;
  int rotationLineWidth = arrowLineWidth;
  int xArrowLength = 80;
  int yArrowLength = 70;
  int zArrowLength = 80;
  int border = rotationDiameter / 2 + rotationLineWidth;
  int editWidth = 60;
  int editHeight = 25;
  int editDistance = 5;
  QPoint axisOrigin(border + editWidth, height() - (border + editHeight/2));
  QPoint xAxisTip(axisOrigin.x() + xArrowLength, axisOrigin.y());
  QPoint yAxisTip(axisOrigin.x() + yArrowLength, axisOrigin.y() - yArrowLength);
  QPoint zAxisTip(axisOrigin.x(), axisOrigin.y() - zArrowLength);

  PaintPrimitives::drawArrow(painter, axisOrigin, xAxisTip, arrowLineWidth, xAxisColor);
  PaintPrimitives::drawArrow(painter, axisOrigin, yAxisTip, arrowLineWidth, yAxisColor);
  PaintPrimitives::drawArrow(painter, axisOrigin, zAxisTip, arrowLineWidth, zAxisColor);

  QPoint xAxisRotationCenter(axisOrigin.x() + (3*xArrowLength)/4, axisOrigin.y());
  double xAxisRotationAngle = 0;
  PaintPrimitives::drawRotation(painter, xAxisRotationCenter, xAxisRotationAngle, rotationDiameter, rotationLineWidth, xAxisColor);

  QPoint yAxisRotationCenter(axisOrigin.x() + (3*yArrowLength)/4, axisOrigin.y() - (3*yArrowLength)/4);
  double yAxisRotationAngle = 45;
  PaintPrimitives::drawRotation(painter, yAxisRotationCenter, yAxisRotationAngle, rotationDiameter, rotationLineWidth, yAxisColor);

  QPoint zAxisRotationCenter(axisOrigin.x(), axisOrigin.y() - (3*zArrowLength)/4);
  double zAxisRotationAngle = 90;
  PaintPrimitives::drawRotation(painter, zAxisRotationCenter, zAxisRotationAngle, rotationDiameter, rotationLineWidth, zAxisColor);

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
