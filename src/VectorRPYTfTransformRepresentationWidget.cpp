#include "VectorRPYTfTransformRepresentationWidget.h"

// system includes
#include <iostream>

// library includes
#include <QMessageBox>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
VectorRPYTfTransformRepresentationWidget::VectorRPYTfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent)
  :TfTransformRepresentationWidget(p_tf, p_parent)
{
  createGraphicFrame();
  updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
VectorRPYTfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  TfTransformRepresentationWidget::setReadOnly(p_ro);

  //m_graphicWidget->forallEdits->setReadOnly(p_ro);
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
void
VectorRPYTfTransformRepresentationWidget::updateTransform()
{
  /*
  tf2::Vector3 translationVector;
  for (unsigned j = 0; j < 3; j++) {
    translationVector[j] = m_graphicWidget->matrixEdits[3][j]->text().toDouble();
  }
  m_tf->setOrigin(translationVector);
  */
}

void
VectorRPYTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;


  //tf2::Vector3 translationVector = m_tf->getOrigin();
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
VectorRPYTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicWidget = new VectorRPYGraphicWidget();
  m_topLayout->insertWidget(0, m_graphicWidget);
  //connect(m_graphicWidget->forallEdits, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
