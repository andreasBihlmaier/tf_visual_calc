#include "TfTransformWidget.h"

// system includes

// library includes
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QLabel>

// custom includes
#include "TfTransformRepSelectionWidget.h"

/*---------------------------------- public: -----------------------------{{{-*/
TfTransformWidget::TfTransformWidget(QWidget* p_parent)
  :QWidget(p_parent),
   m_broadcastCount(0)
{
  m_tf.setOrigin(tf2::Vector3(0.0, 2.0, 0.0));
  tf2::Quaternion quat;
  quat.setEuler(0, 0, 0);
  m_tf.setRotation(quat);

  createLayout();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
void
TfTransformWidget::broadcastTransform()
{
  geometry_msgs::TransformStamped tfStampedMsg;
  tfStampedMsg.header.seq = m_broadcastCount++;
  tfStampedMsg.header.stamp = ros::Time::now();
  tfStampedMsg.header.frame_id = m_tfParent;

  // tf2 sucks
  tfStampedMsg.transform.translation.x = m_tf.getOrigin().getX();
  tfStampedMsg.transform.translation.y = m_tf.getOrigin().getY();
  tfStampedMsg.transform.translation.z = m_tf.getOrigin().getZ();
  tfStampedMsg.transform.rotation.x = m_tf.getRotation().getX();
  tfStampedMsg.transform.rotation.y = m_tf.getRotation().getY();
  tfStampedMsg.transform.rotation.z = m_tf.getRotation().getZ();
  tfStampedMsg.transform.rotation.w = m_tf.getRotation().getW();

  tfStampedMsg.child_frame_id = m_tfName;

  m_tfBroadcaster.sendTransform(tfStampedMsg);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfTransformWidget::createLayout()
{
  m_tfNameLabel = new QLabel("tfName:");
  m_tfNameEdit = new QLineEdit();
  connect(m_tfNameEdit, SIGNAL(textChanged(const QString&)), this, SLOT(setTfName(const QString&)));

  m_tfNameLayout = new QHBoxLayout();
  m_tfNameLayout->addWidget(m_tfNameLabel);
  m_tfNameLayout->addWidget(m_tfNameEdit);

  m_relativeLabel = new QLabel("relative:");
  m_relativeRepSelectionWidget = new TfTransformRepSelectionWidget();
  m_relativeRepSelectionWidget->setTransform(&m_tf);

  m_absoluteLabel = new QLabel("absolute:");
  m_absoluteRepSelectionWidget = new TfTransformRepSelectionWidget();
  m_absoluteRepSelectionWidget->setReadOnly(true);
  m_absoluteRepSelectionWidget->setTransform(&m_absoluteTf);


  m_topLayout = new QVBoxLayout();
  m_topLayout->addLayout(m_tfNameLayout);
  m_topLayout->addWidget(m_relativeLabel);
  m_topLayout->addWidget(m_relativeRepSelectionWidget);
  m_topLayout->addSpacing(2);
  m_topLayout->addWidget(m_absoluteLabel);
  m_topLayout->addWidget(m_absoluteRepSelectionWidget);
  setLayout(m_topLayout);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
void
TfTransformWidget::setTfName(const QString& p_name)
{
  m_tfName = p_name.toStdString();
}

void
TfTransformWidget::setTfParent(const QString& p_parent)
{
  m_tfParent = p_parent.toStdString();
}
/*------------------------------------------------------------------------}}}-*/
