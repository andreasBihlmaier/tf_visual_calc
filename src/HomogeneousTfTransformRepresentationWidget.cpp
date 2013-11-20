#include "HomogeneousTfTransformRepresentationWidget.h"

// system includes
#include <iostream>

// library includes

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
  std::cout << "updateTransform" << std::endl;
}

void
HomogeneousTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;

  tf2::Matrix3x3 rotationMatrix = m_tf->getBasis();
  for (unsigned i = 0; i < 3; i++) {
    for (unsigned j = 0; j < 3; j++) {
      setEditNumberNoSignal(m_graphicWidget->matrixEdits[i][j], rotationMatrix[i][j]);
    }
  }
  for (unsigned i = 0; i < 3; i++) {
    setEditNumberNoSignal(m_graphicWidget->matrixEdits[i][3], 0);
  }
  setEditNumberNoSignal(m_graphicWidget->matrixEdits[3][3], 1);

  tf2::Vector3 translationVector = m_tf->getOrigin();
  for (unsigned j = 0; j < 3; j++) {
    setEditNumberNoSignal(m_graphicWidget->matrixEdits[3][j], translationVector[j]);
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
      connect(m_graphicWidget->matrixEdits[i][j], SIGNAL(textChanged(const QString&)), this, SLOT(updateTransform()));
    }
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
