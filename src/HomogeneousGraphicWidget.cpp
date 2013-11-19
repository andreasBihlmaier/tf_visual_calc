#include "HomogeneousGraphicWidget.h"

// system includes

// library includes
#include <QPainter>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
HomogeneousGraphicWidget::HomogeneousGraphicWidget(QWidget* p_parent)
  :QWidget(p_parent)
{
  setMinimumHeight(100);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- protected: ----------------------------{{{-*/
void
HomogeneousGraphicWidget::paintEvent(QPaintEvent* p_event)
{
  QPainter painter(this);
  QFont font;
  font.setPointSize(60);
  painter.setFont(font);
  painter.drawText(0, height()/2, "[");
  //painter.drawText(width() - 40, height(), "]");
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
