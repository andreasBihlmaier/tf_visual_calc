#include "HomogeneousTfTransformRepresentationWidget.h"

// system includes
#include <iostream>

// library includes
#include <QMessageBox>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
HomogeneousTfTransformRepresentationWidget::HomogeneousTfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent)
  :TfTransformRepresentationWidget(p_tf, p_parent)
{
  createGraphicFrame();
  updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  TfTransformRepresentationWidget::setReadOnly(p_ro);

  for (unsigned row = 0; row < 4; row++) {
    for (unsigned col = 0; col < 4; col++) {
      m_graphicWidget->m_matrixEdits[row][col]->setReadOnly(p_ro);
    }
  }
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::updateTransformFromGraphic()
{
  for (unsigned row = 0; row < 4; row++) {
    for (unsigned col = 0; col < 4; col++) {
      if (m_graphicWidget->m_matrixEdits[row][col]->text().isEmpty())
        return;
    }
  }

  for (unsigned col = 0; col < 3; col++) {
    if (m_graphicWidget->m_matrixEdits[3][col]->text().toDouble() != 0) {
      QMessageBox::critical(this, "Homogeneous Representation",
                            QString("Field (%1, %2) must be %3").arg(3).arg(col).arg(0));
      m_graphicWidget->m_matrixEdits[3][col]->setText(number(0));
      return;
    }
  }
  if (m_graphicWidget->m_matrixEdits[3][3]->text().toDouble() != 1) {
    QMessageBox::critical(this, "Homogeneous Representation",
                          QString("Field (%1, %2) must be %3").arg(3).arg(3).arg(1));
      m_graphicWidget->m_matrixEdits[3][3]->setText(number(1));
    return;
  }


  tf2::Matrix3x3 rotationMatrix;
  for (unsigned row = 0; row < 3; row++) {
    for (unsigned col = 0; col < 3; col++) {
       rotationMatrix[row][col] = m_graphicWidget->m_matrixEdits[row][col]->text().toDouble();
    }
  }
  m_tf->setBasis(rotationMatrix);

  tf2::Vector3 translationVector;
  for (unsigned row = 0; row < 3; row++) {
    translationVector[row] = m_graphicWidget->m_matrixEdits[row][3]->text().toDouble();
  }
  m_tf->setOrigin(translationVector);
}

void
HomogeneousTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;

  tf2::Matrix3x3 rotationMatrix = m_tf->getBasis();
  for (unsigned row = 0; row < 3; row++) {
    for (unsigned col = 0; col < 3; col++) {
      m_graphicWidget->m_matrixEdits[row][col]->setText(number(rotationMatrix[row][col]));
    }
  }
  for (unsigned col = 0; col < 3; col++) {
    m_graphicWidget->m_matrixEdits[3][col]->setText(number(0));
  }
  m_graphicWidget->m_matrixEdits[3][3]->setText(number(1));

  tf2::Vector3 translationVector = m_tf->getOrigin();
  for (unsigned row = 0; row < 3; row++) {
    m_graphicWidget->m_matrixEdits[row][3]->setText(number(translationVector[row]));
  }
}

void
HomogeneousTfTransformRepresentationWidget::updateTransformFromText()
{
  // TODO
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicWidget = new HomogeneousGraphicWidget();
  m_topLayout->insertWidget(0, m_graphicWidget);
  for (unsigned row = 0; row < 4; row++) {
    for (unsigned col = 0; col < 4; col++) {
      connect(m_graphicWidget->m_matrixEdits[row][col], SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
    }
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
