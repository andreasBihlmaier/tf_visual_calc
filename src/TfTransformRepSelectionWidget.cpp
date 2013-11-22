#include "TfTransformRepSelectionWidget.h"

// system includes

// library includes
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QComboBox>

#include <tf2/LinearMath/Transform.h>

// custom includes
#include "HomogeneousTfTransformRepresentationWidget.h"

/*---------------------------------- public: -----------------------------{{{-*/
TfTransformRepSelectionWidget::TfTransformRepSelectionWidget(QWidget* p_parent)
  :QWidget(p_parent),
   m_tf(NULL),
   m_readOnly(false)
{
  createLayout();
}

void
TfTransformRepSelectionWidget::setTransform(tf2::Transform* p_tf)
{
  m_tf = p_tf;
  m_representationWidget->setTransform(m_tf);
  m_representationWidget->updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
TfTransformRepSelectionWidget::setReadOnly(bool p_ro)
{
  m_readOnly = p_ro;
  m_representationWidget->setReadOnly(p_ro);
}

void
TfTransformRepSelectionWidget::setRepresentation(int p_representation)
{
  switch (p_representation) {
  case HomogeneousRepresentation:
    m_representationWidget = new HomogeneousTfTransformRepresentationWidget(m_tf);
    break;
  case VectorRPYRepresentation:
    break;
  case VectorQuaternionRepresentation:
    break;
  case DenavitHartenbergRepresentation:
    break;
  }
}

void
TfTransformRepSelectionWidget::updateDisplay()
{
  m_representationWidget->updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfTransformRepSelectionWidget::createLayout()
{
  m_representationLabel = new QLabel("Representation:");
  m_representationComboBox = new QComboBox();
  m_representationComboBox->insertItem(HomogeneousRepresentation, "Homogeneous");
  m_representationComboBox->insertItem(VectorRPYRepresentation, "Vector + RPY");
  m_representationComboBox->insertItem(VectorQuaternionRepresentation, "Vector + Quaternion");
  m_representationComboBox->insertItem(DenavitHartenbergRepresentation, "Denavit-Hartenberg");
  connect(m_representationComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(setRepresentation(int)));
  setRepresentation(m_representationComboBox->currentIndex());

  m_representationLayout = new QHBoxLayout();
  m_representationLayout->addWidget(m_representationLabel);
  m_representationLayout->addWidget(m_representationComboBox);

  m_topLayout = new QVBoxLayout();
  m_topLayout->addLayout(m_representationLayout);
  m_topLayout->addWidget(m_representationWidget);
  setLayout(m_topLayout);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
