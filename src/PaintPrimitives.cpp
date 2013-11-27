#include "PaintPrimitives.h"

// system includes
#include <cmath>

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
void
PaintPrimitives::drawArrow(QPainter& p_painter, const QPoint& p_origin, const QPoint& p_tip, int p_lineWidth, const QColor& p_color)
{
  p_painter.save();

  QPen pen(p_color);
  pen.setWidth(p_lineWidth);
  p_painter.setPen(pen);

  p_painter.drawLine(p_origin, p_tip);

  double lineAngle = atan2(p_tip.y() - p_origin.y(), p_tip.x() - p_origin.x()) * 180.0/M_PI;
  p_painter.translate(p_tip);
  p_painter.rotate(lineAngle);
  p_painter.drawLine(QPoint(), QPoint(-p_lineWidth * 2, -p_lineWidth * 2));
  p_painter.drawLine(QPoint(), QPoint(-p_lineWidth * 2, p_lineWidth * 2));

  p_painter.restore();
}

void
PaintPrimitives::drawRotation(QPainter& p_painter, const QPoint& p_center, double p_angle, int p_diameter, int p_lineWidth, const QColor& p_color)
{
  p_painter.save();

  int pixmapLen = p_diameter;
  QPixmap rotationPixmap(pixmapLen, pixmapLen);
  rotationPixmap.fill(QColor(0, 0, 0, 0));
  QPainter pixmapPainter(&rotationPixmap);
  QPen pen(p_color);
  pen.setWidth(p_lineWidth);
  pixmapPainter.setPen(pen);

  int pixmapDiameter = p_diameter - p_lineWidth;
  pixmapPainter.drawArc(QRect(0, 0, pixmapDiameter, pixmapDiameter), 16 * 45, 16 * 270);
  drawArrow(pixmapPainter, QPoint(pixmapDiameter/2 + pixmapDiameter/6, pixmapDiameter), QPoint(pixmapDiameter, pixmapDiameter/2 + pixmapDiameter/4), p_lineWidth, p_color);


  QSize targetSize = rotationPixmap.size();
  if (p_angle < 30 || p_angle > 60) {
    targetSize.scale(targetSize.width() - targetSize.width()/2 * cos(p_angle), targetSize.width() - targetSize.height()/2 * sin(p_angle), Qt::IgnoreAspectRatio);
  }
  p_painter.drawPixmap(QRect(p_center - QPoint(targetSize.width()/2, targetSize.height()/2), targetSize), rotationPixmap, rotationPixmap.rect());

  p_painter.restore();
}

std::vector<QPoint>
PaintPrimitives::drawCoordinateAxis(QPainter& p_painter, const QPoint& p_origin, int p_arrowLineWidth, bool p_drawRotation, int p_rotationDiameter)
{
  p_painter.save();

  QColor xAxisColor("red");
  QColor yAxisColor("green");
  QColor zAxisColor("blue");
  int rotationLineWidth = p_arrowLineWidth;
  int xArrowLength = 80;
  int yArrowLength = 70;
  int zArrowLength = 80;
  QPoint xAxisTip(p_origin.x() + xArrowLength, p_origin.y());
  QPoint yAxisTip(p_origin.x() + yArrowLength, p_origin.y() - yArrowLength);
  QPoint zAxisTip(p_origin.x(), p_origin.y() - zArrowLength);

  PaintPrimitives::drawArrow(p_painter, p_origin, xAxisTip, p_arrowLineWidth, xAxisColor);
  PaintPrimitives::drawArrow(p_painter, p_origin, yAxisTip, p_arrowLineWidth, yAxisColor);
  PaintPrimitives::drawArrow(p_painter, p_origin, zAxisTip, p_arrowLineWidth, zAxisColor);

  std::vector<QPoint> arrowTips;
  arrowTips.push_back(xAxisTip);
  arrowTips.push_back(yAxisTip);
  arrowTips.push_back(zAxisTip);

  if (p_drawRotation) {
    QPoint xAxisRotationCenter(p_origin.x() + (3*xArrowLength)/4, p_origin.y());

    double xAxisRotationAngle = 0;
    PaintPrimitives::drawRotation(p_painter, xAxisRotationCenter, xAxisRotationAngle, p_rotationDiameter, rotationLineWidth, xAxisColor);

    QPoint yAxisRotationCenter(p_origin.x() + (3*yArrowLength)/4, p_origin.y() - (3*yArrowLength)/4);
    double yAxisRotationAngle = 45;
    PaintPrimitives::drawRotation(p_painter, yAxisRotationCenter, yAxisRotationAngle, p_rotationDiameter, rotationLineWidth, yAxisColor);

    QPoint zAxisRotationCenter(p_origin.x(), p_origin.y() - (3*zArrowLength)/4);
    double zAxisRotationAngle = 90;
    PaintPrimitives::drawRotation(p_painter, zAxisRotationCenter, zAxisRotationAngle, p_rotationDiameter, rotationLineWidth, zAxisColor);

    arrowTips.push_back(xAxisRotationCenter);
    arrowTips.push_back(yAxisRotationCenter);
    arrowTips.push_back(zAxisRotationCenter);
  }

  p_painter.restore();
  return arrowTips;
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
