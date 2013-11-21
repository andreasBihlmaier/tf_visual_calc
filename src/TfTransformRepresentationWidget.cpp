#include "TfTransformRepresentationWidget.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
TfTransformRepresentationWidget::TfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent)
  :QWidget(p_parent),
   m_tf(p_tf)
{
  createLayout();
}

void
TfTransformRepresentationWidget::setTransform(tf2::Transform* p_tf)
{
  m_tf = p_tf;
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
void
TfTransformRepresentationWidget::createLayout()
{
  m_textEdit = new QPlainTextEdit();

  m_topLayout = new QHBoxLayout();
  m_topLayout->addWidget(m_textEdit);
  setLayout(m_topLayout);
}
/*------------------------------------------------------------------------}}}-*/
