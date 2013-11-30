#include "TfTransformRepresentationWidget.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
QString
TfTransformRepresentationWidget::number(double p_num)
{
  return QString::number(p_num, 'f', 4);
}

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

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
TfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  m_textEdit->setReadOnly(p_ro);
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
void
TfTransformRepresentationWidget::setText(const QString& p_text)
{
  bool oldState = m_textEdit->blockSignals(true);
  m_textEdit->setPlainText(p_text);
  m_textEdit->blockSignals(oldState);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
void
TfTransformRepresentationWidget::createLayout()
{
  m_textEdit = new QPlainTextEdit();
  m_textEdit->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  connect(m_textEdit, SIGNAL(textChanged()), this, SLOT(updateTransformFromText()));

  m_topLayout = new QHBoxLayout();
  m_topLayout->addWidget(m_textEdit);
  setLayout(m_topLayout);
}
/*------------------------------------------------------------------------}}}-*/
