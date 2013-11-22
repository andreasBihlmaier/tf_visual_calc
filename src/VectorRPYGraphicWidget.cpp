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
  setFixedHeight(100);

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
  int arrowLineWidth = 3;
  int xArrowLength = 90;
  int yArrowLength = 70;
  int zArrowLength = 90;
  int border = 2 * arrowLineWidth + (arrowLineWidth+1)/2;
  QPoint axisOrigin(border, height() - border);
  QPoint xAxisTip(axisOrigin.x() + xArrowLength, axisOrigin.y());
  QPoint yAxisTip(axisOrigin.x() + yArrowLength, axisOrigin.y() - yArrowLength);
  QPoint zAxisTip(axisOrigin.x(), axisOrigin.y() - zArrowLength);

  PaintPrimitives::drawArrow(painter, axisOrigin, xAxisTip, arrowLineWidth, QColor("red"));
  PaintPrimitives::drawArrow(painter, axisOrigin, yAxisTip, arrowLineWidth, QColor("green"));
  PaintPrimitives::drawArrow(painter, axisOrigin, zAxisTip, arrowLineWidth, QColor("blue"));


  /*
  QFont font;
  font.setPointSize(68);
  int borderWidth = 30;
  painter.setFont(font);
  int textheight = height() - 12;
  painter.drawText(0, textheight, "[");
  painter.drawText(width() - borderWidth, textheight, "]");
  int widthWithoutBorder = width() - (borderWidth*2);
  int xw = widthWithoutBorder / 4;
  int yw = height() / 4;
  int verticalLineX = borderWidth + xw * 3 - 1;
  painter.drawLine(verticalLineX, 0, verticalLineX, height());
  int horizontalLineY = yw * 3;
  painter.drawLine(borderWidth, horizontalLineY, widthWithoutBorder + 4 * 6, horizontalLineY);

  matrixEdits[i][j]->setGeometry(borderWidth + i * xw, j * yw, xw, yw);
  */
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
VectorRPYGraphicWidget::createChildWidgets()
{
  /*
  matrixEdits[i][j] = new QLineEdit(this);
  matrixEdits[i][j]->setValidator(new QDoubleValidator());
  */
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
