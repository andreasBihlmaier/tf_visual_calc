#include "HomogeneousTfTransformRepresentationWidget.h"

// system includes
#include <iostream>

// library includes
#include <QMessageBox>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
HomogeneousTfTransformRepresentationWidget::HomogeneousTfTransformRepresentationWidget(QWidget* p_parent, tf2::Transform* p_tf)
  :TfTransformRepresentationWidget(p_parent, p_tf)
{
  createGraphicFrame();
  updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  TfTransformRepresentationWidget::setReadOnly(p_ro);

  for (unsigned i = 0; i < 4; i++) {
    for (unsigned j = 0; j < 4; j++) {
      m_graphicWidget->matrixEdits[i][j]->setReadOnly(p_ro);
    }
  }
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected slots: ------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::updateTransform()
{
  for (unsigned i = 0; i < 4; i++) {
    for (unsigned j = 0; j < 4; j++) {
      if (m_graphicWidget->matrixEdits[i][j]->text().isEmpty())
        return;
    }
  }

  for (unsigned i = 0; i < 3; i++) {
    if (m_graphicWidget->matrixEdits[i][3]->text().toDouble() != 0) {
      QMessageBox::critical(this, "Homogeneous Representation",
                            QString("Field (%1, %2) must be %3").arg(i).arg(3).arg(0));
      return;
    }
  }
  if (m_graphicWidget->matrixEdits[3][3]->text().toDouble() != 1) {
    QMessageBox::critical(this, "Homogeneous Representation",
                          QString("Field (%1, %2) must be %3").arg(3).arg(3).arg(1));
    return;
  }


  tf2::Matrix3x3 rotationMatrix;
  for (unsigned i = 0; i < 3; i++) {
    for (unsigned j = 0; j < 3; j++) {
       rotationMatrix[i][j] = m_graphicWidget->matrixEdits[i][j]->text().toDouble();
    }
  }
  m_tf->setBasis(rotationMatrix);

  tf2::Vector3 translationVector;
  for (unsigned j = 0; j < 3; j++) {
    translationVector[j] = m_graphicWidget->matrixEdits[3][j]->text().toDouble();
  }
  m_tf->setOrigin(translationVector);
}

void
HomogeneousTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;

  tf2::Matrix3x3 rotationMatrix = m_tf->getBasis();
  for (unsigned i = 0; i < 3; i++) {
    for (unsigned j = 0; j < 3; j++) {
      m_graphicWidget->matrixEdits[i][j]->setText(QString::number(rotationMatrix[i][j]));
    }
  }
  for (unsigned i = 0; i < 3; i++) {
    m_graphicWidget->matrixEdits[i][3]->setText(QString::number(0));
  }
  m_graphicWidget->matrixEdits[3][3]->setText(QString::number(1));

  tf2::Vector3 translationVector = m_tf->getOrigin();
  for (unsigned j = 0; j < 3; j++) {
    m_graphicWidget->matrixEdits[3][j]->setText(QString::number(translationVector[j]));
  }
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
HomogeneousTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicWidget = new HomogeneousGraphicWidget();
  m_topLayout->insertWidget(0, m_graphicWidget);
  for (unsigned i = 0; i < 4; i++) {
    for (unsigned j = 0; j < 4; j++) {
      connect(m_graphicWidget->matrixEdits[i][j], SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
    }
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
