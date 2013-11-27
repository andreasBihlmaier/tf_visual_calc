#include "VectorQuaternionTfTransformRepresentationWidget.h"

// system includes
#include <iostream>

// library includes
#include <QMessageBox>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
VectorQuaternionTfTransformRepresentationWidget::VectorQuaternionTfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent)
  :TfTransformRepresentationWidget(p_tf, p_parent)
{
  createGraphicFrame();
  updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
VectorQuaternionTfTransformRepresentationWidget::setReadOnly(bool p_ro)
{
  TfTransformRepresentationWidget::setReadOnly(p_ro);

  m_graphicWidget->m_xEdit->setReadOnly(p_ro);
  m_graphicWidget->m_yEdit->setReadOnly(p_ro);
  m_graphicWidget->m_zEdit->setReadOnly(p_ro);
  m_graphicWidget->m_qxEdit->setReadOnly(p_ro);
  m_graphicWidget->m_qyEdit->setReadOnly(p_ro);
  m_graphicWidget->m_qzEdit->setReadOnly(p_ro);
  m_graphicWidget->m_qwEdit->setReadOnly(p_ro);
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
void
VectorQuaternionTfTransformRepresentationWidget::updateTransform()
{
  tf2::Quaternion quaternion;
  quaternion.setX(m_graphicWidget->m_qxEdit->text().toDouble());
  quaternion.setY(m_graphicWidget->m_qyEdit->text().toDouble());
  quaternion.setZ(m_graphicWidget->m_qzEdit->text().toDouble());
  quaternion.setW(m_graphicWidget->m_qwEdit->text().toDouble());
  m_tf->setRotation(quaternion);
  tf2::Vector3 translationVector(m_graphicWidget->m_xEdit->text().toDouble(),
                                 m_graphicWidget->m_yEdit->text().toDouble(),
                                 m_graphicWidget->m_zEdit->text().toDouble());
  m_tf->setOrigin(translationVector);
}

void
VectorQuaternionTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;

  tf2::Quaternion quaternion = m_tf->getRotation();
  m_graphicWidget->m_qxEdit->setText(QString::number(quaternion.getX()));
  m_graphicWidget->m_qyEdit->setText(QString::number(quaternion.getY()));
  m_graphicWidget->m_qzEdit->setText(QString::number(quaternion.getZ()));
  m_graphicWidget->m_qwEdit->setText(QString::number(quaternion.getW()));

  tf2::Vector3 translationVector = m_tf->getOrigin();
  m_graphicWidget->m_xEdit->setText(QString::number(translationVector.x()));
  m_graphicWidget->m_yEdit->setText(QString::number(translationVector.y()));
  m_graphicWidget->m_zEdit->setText(QString::number(translationVector.z()));
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
VectorQuaternionTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicWidget = new VectorQuaternionGraphicWidget();
  m_topLayout->insertWidget(0, m_graphicWidget);
  connect(m_graphicWidget->m_xEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_yEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_zEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_qxEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_qyEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_qzEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
  connect(m_graphicWidget->m_qwEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransform()));
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
