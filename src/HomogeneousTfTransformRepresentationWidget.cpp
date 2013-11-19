#include "HomogeneousTfTransformRepresentationWidget.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
HomogeneousTfTransformRepresentationWidget::HomogeneousTfTransformRepresentationWidget(QWidget* p_parent, tf2::Transform* p_tf)
  :TfTransformRepresentationWidget(p_parent, p_tf)
{
  createGraphicFrame();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  TfTransformRepresentationWidget::setReadOnly(p_ro);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- protected: ----------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::paintEvent(QPaintEvent* p_event)
{
  m_graphicPainter->drawLine(0, 0, 10, 30);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicFrame->setMinimumHeight(50);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
