#include "TfTransformRepresentationWidget.h"

// system includes

// library includes
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include <tf2/LinearMath/Transform.h>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
TfTransformRepresentationWidget::TfTransformRepresentationWidget(QWidget* p_parent, tf2::Transform* p_tf)
  :QWidget(p_parent),
   m_tf(p_tf)
{
  createLayout();
  m_graphicPainter = new QPainter(m_graphicFrame);
}

void
TfTransformRepresentationWidget::createLayout()
{
  m_graphicFrame = new QFrame();
  m_textEdit = new QLineEdit();

  m_topLayout = new QVBoxLayout();
  m_topLayout->addWidget(m_graphicFrame);
  m_topLayout->addWidget(m_textEdit);
  setLayout(m_topLayout);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
void
TfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  m_textEdit->setReadOnly(p_ro);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
