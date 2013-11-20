#include "HomogeneousGraphicWidget.h"

// system includes

// library includes
#include <QPainter>
#include <QDoubleValidator>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
HomogeneousGraphicWidget::HomogeneousGraphicWidget(QWidget* p_parent)
  :QWidget(p_parent)
{
  setMinimumWidth(300);
  setFixedHeight(80);

  createChildWidgets();
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

  for (unsigned i = 0; i < 4; i++) {
    for (unsigned j = 0; j < 4; j++) {
      matrixEdits[i][j]->setGeometry(borderWidth + i * xw, j * yw, xw, yw);
    }
  }
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
HomogeneousGraphicWidget::createChildWidgets()
{
  for (unsigned i = 0; i < 4; i++) {
    for (unsigned j = 0; j < 4; j++) {
      matrixEdits[i][j] = new QLineEdit(this);
      matrixEdits[i][j]->setValidator(new QDoubleValidator());
    }
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
