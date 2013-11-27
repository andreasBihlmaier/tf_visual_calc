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

  m_graphicWidget->m_xEdit->setReadOnly(p_ro);
  m_graphicWidget->m_yEdit->setReadOnly(p_ro);
  m_graphicWidget->m_zEdit->setReadOnly(p_ro);
  m_graphicWidget->m_rxEdit->setReadOnly(p_ro);
  m_graphicWidget->m_ryEdit->setReadOnly(p_ro);
  m_graphicWidget->m_rzEdit->setReadOnly(p_ro);
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
void
VectorRPYTfTransformRepresentationWidget::updateTransform()
{
  tf2::Matrix3x3 rotationMatrix;
  rotationMatrix.setRPY(m_graphicWidget->m_rxEdit->text().toDouble(),
                        m_graphicWidget->m_ryEdit->text().toDouble(),
                        m_graphicWidget->m_rzEdit->text().toDouble());
  m_tf->setBasis(rotationMatrix);
  tf2::Vector3 translationVector(m_graphicWidget->m_xEdit->text().toDouble(),
                                 m_graphicWidget->m_yEdit->text().toDouble(),
                                 m_graphicWidget->m_zEdit->text().toDouble());
  m_tf->setOrigin(translationVector);
}

void
VectorRPYTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;

  tf2::Matrix3x3 rotationMatrix = m_tf->getBasis();
  double roll;
  double pitch;
  double yaw;
  rotationMatrix.getRPY(roll, pitch, yaw);
  m_graphicWidget->m_rxEdit->setText(number(roll));
  m_graphicWidget->m_ryEdit->setText(number(pitch));
  m_graphicWidget->m_rzEdit->setText(number(yaw));

  tf2::Vector3 translationVector = m_tf->getOrigin();
  m_graphicWidget->m_xEdit->setText(number(translationVector.x()));
  m_graphicWidget->m_yEdit->setText(number(translationVector.y()));
  m_graphicWidget->m_zEdit->setText(number(translationVector.z()));
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
VectorRPYTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicWidget = new VectorRPYGraphicWidget();
  m_topLayout->insertWidget(0, m_graphicWidget);
  connect(m_graphicWidget->m_xEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_yEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_zEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_rxEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_ryEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_rzEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
