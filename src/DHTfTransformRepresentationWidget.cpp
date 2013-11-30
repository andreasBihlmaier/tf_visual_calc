#include "DHTfTransformRepresentationWidget.h"

// system includes
#include <iostream>

// library includes
#include <QMessageBox>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
tf2::Transform
DHTfTransformRepresentationWidget::dh2Transform(double p_d, double p_theta, double p_a, double p_alpha)
{
  tf2::Transform dhTransform;

  tf2::Matrix3x3 rotationMatrix;
  rotationMatrix[0][0] = cos(p_theta);
  rotationMatrix[0][1] = -sin(p_theta) * cos(p_alpha);
  rotationMatrix[0][2] = sin(p_theta) * sin(p_alpha);
  rotationMatrix[1][0] = sin(p_theta);
  rotationMatrix[1][1] = cos(p_theta) * cos(p_alpha);
  rotationMatrix[1][2] = - cos(p_theta) * sin(p_alpha);
  rotationMatrix[2][0] = 0;
  rotationMatrix[2][1] = sin(p_alpha);
  rotationMatrix[2][2] = cos(p_alpha);
  dhTransform.setBasis(rotationMatrix);

  tf2::Vector3 translationVector(p_a * cos(p_theta),
                                 p_a * sin(p_theta),
                                 p_d);
  dhTransform.setOrigin(translationVector);

  return dhTransform;
}

bool
DHTfTransformRepresentationWidget::isEqual(const tf2::Transform& p_tfA, const tf2::Transform& p_tfB)
{
  double equalEpsilon = 1e-6;
  double angle = p_tfA.getRotation().angle(p_tfB.getRotation());
  double dist = (p_tfA.getOrigin() - p_tfB.getOrigin()).length();

  if (abs(angle) > equalEpsilon || dist > equalEpsilon) {
    return false;
  }

  return true;
}

DHTfTransformRepresentationWidget::DHTfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent)
  :TfTransformRepresentationWidget(p_tf, p_parent)
{
  createGraphicFrame();
  updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
DHTfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  TfTransformRepresentationWidget::setReadOnly(p_ro);

  m_graphicWidget->m_dEdit->setReadOnly(p_ro);
  m_graphicWidget->m_aEdit->setReadOnly(p_ro);
  m_graphicWidget->m_thetaEdit->setReadOnly(p_ro);
  m_graphicWidget->m_alphaEdit->setReadOnly(p_ro);
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
void
DHTfTransformRepresentationWidget::updateTransformFromGraphic()
{
  tf2::Transform dhTransform = dh2Transform(m_graphicWidget->m_dEdit->text().toDouble(),
                                            m_graphicWidget->m_thetaEdit->text().toDouble(),
                                            m_graphicWidget->m_aEdit->text().toDouble(),
                                            m_graphicWidget->m_alphaEdit->text().toDouble());
  m_tf->setRotation(dhTransform.getRotation());
  m_tf->setOrigin(dhTransform.getOrigin());
}

void
DHTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;

  tf2::Matrix3x3 rotationMatrix = m_tf->getBasis();
  tf2::Vector3 translationVector = m_tf->getOrigin();
  double d = translationVector.getZ();
  double cosTheta = rotationMatrix[0][0];
  double theta = acos(cosTheta);
  double a = translationVector.getX() / cosTheta;
  double sinAlpha = rotationMatrix[1][2];
  double alpha = asin(sinAlpha);

  tf2::Transform shouldDHTransform = dh2Transform(d, theta, a, alpha);
  // check whether transform can be represented as DH
  if (!isEqual(*m_tf, shouldDHTransform)) {
    QMessageBox::critical(this, "Denavit-Hartenberg Representation",
                          QString("Transform can't be represented as DH parameters"));
    return;
  }

  m_graphicWidget->m_aEdit->setText(number(a));
  m_graphicWidget->m_dEdit->setText(number(d));
  m_graphicWidget->m_thetaEdit->setText(number(theta));
  m_graphicWidget->m_alphaEdit->setText(number(alpha));
}

void
DHTfTransformRepresentationWidget::updateTransformFromText()
{
  // TODO
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
DHTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicWidget = new DHGraphicWidget();
  m_topLayout->insertWidget(0, m_graphicWidget);
  connect(m_graphicWidget->m_dEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_aEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_thetaEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_alphaEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
