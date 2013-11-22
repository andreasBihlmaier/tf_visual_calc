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
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
