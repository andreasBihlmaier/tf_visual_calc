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
  setMinimumHeight(160);

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
VectorQuaternionTfTransformRepresentationWidget::updateTransformFromGraphic()
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
  updateTextDisplay();
}

void
VectorQuaternionTfTransformRepresentationWidget::updateDisplay()
{
  if (m_tf == NULL)
    return;

  updateGraphicDisplay();
  updateTextDisplay();
}

void
VectorQuaternionTfTransformRepresentationWidget::updateTransformFromText()
{
  std::string text = m_textEdit->toPlainText().toStdString();
  //std::cout << text << std::endl;
  double tx, ty, tz, qx, qy, qz, qw;
  int matchCount;
  if ((matchCount = sscanf(text.c_str(), "position:\n  x: %lf\n  y: %lf\n  z: %lf\norientation:\n  x: %lf\n  y: %lf\n  z: %lf\n  w: %lf\n", &tx, &ty, &tz, &qx, &qy, &qz, &qw)) == 7) {
    //std::cout << "VALID" << std::endl;
    tf2::Vector3 translation(tx, ty, tz);
    m_tf->setOrigin(translation);
    tf2::Quaternion quaternion(qx, qy, qz, qw);
    m_tf->setRotation(quaternion);
    updateGraphicDisplay();
  } else {
    //std::cout << matchCount << std::endl;
  }
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
VectorQuaternionTfTransformRepresentationWidget::createGraphicFrame()
{
  m_graphicWidget = new VectorQuaternionGraphicWidget();
  m_topLayout->insertWidget(0, m_graphicWidget);
  connect(m_graphicWidget->m_xEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_yEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_zEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_qxEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_qyEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_qzEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
  connect(m_graphicWidget->m_qwEdit, SIGNAL(textEdited(const QString&)), this, SLOT(updateTransformFromGraphic()));
}

void
VectorQuaternionTfTransformRepresentationWidget::updateGraphicDisplay()
{
  tf2::Quaternion quaternion = m_tf->getRotation();
  m_graphicWidget->m_qxEdit->setText(number(quaternion.getX()));
  m_graphicWidget->m_qyEdit->setText(number(quaternion.getY()));
  m_graphicWidget->m_qzEdit->setText(number(quaternion.getZ()));
  m_graphicWidget->m_qwEdit->setText(number(quaternion.getW()));

  tf2::Vector3 translationVector = m_tf->getOrigin();
  m_graphicWidget->m_xEdit->setText(number(translationVector.x()));
  m_graphicWidget->m_yEdit->setText(number(translationVector.y()));
  m_graphicWidget->m_zEdit->setText(number(translationVector.z()));

}

void
VectorQuaternionTfTransformRepresentationWidget::updateTextDisplay()
{
  tf2::Quaternion quaternion = m_tf->getRotation();
  tf2::Vector3 translationVector = m_tf->getOrigin();
  QString asText = QString("position:\n")
                   + QString("  x: %1\n").arg(translationVector.x())
                   + QString("  y: %1\n").arg(translationVector.y())
                   + QString("  z: %1\n").arg(translationVector.z())
                   + QString("orientation:\n")
                   + QString("  x: %1\n").arg(quaternion.getX())
                   + QString("  y: %1\n").arg(quaternion.getY())
                   + QString("  z: %1\n").arg(quaternion.getZ())
                   + QString("  w: %1\n").arg(quaternion.getW());
  setText(asText);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
