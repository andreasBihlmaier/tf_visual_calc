#ifndef _PAINT_PRIMITIVES_H_
#define _PAINT_PRIMITIVES_H_

// system includes

// library includes
#include <QPainter>

// custom includes


// forward declarations



class PaintPrimitives
{
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions
    static void drawArrow(QPainter& p_painter, const QPoint& p_origin, const QPoint& p_tip, int p_lineWidth = 1, const QColor& p_color = QColor("black"));
    static void drawRotation(QPainter& p_painter, const QPoint& p_center, double p_angle, int p_diameter, int p_lineWidth = 1, const QColor& p_color = QColor("black"));


    // constructors

    // overwritten methods

    // methods

    // variables


  protected:
    // methods

    // variables


  private:
    // methods

    // variables


};

#endif // _PAINT_PRIMITIVES_H_
