#include "TfTransformRepresentationWidget.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
void
TfTransformRepresentationWidget::setEditNumberNoSignal(QLineEdit* p_edit, int p_number)
{
  bool oldState = p_edit->blockSignals(true);
  p_edit->setText(QString::number(p_number));
  p_edit->blockSignals(oldState);
}

TfTransformRepresentationWidget::TfTransformRepresentationWidget(QWidget* p_parent, tf2::Transform* p_tf)
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
