#include "TfTransformWidget.h"

// system includes

// library includes
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLineEdit>
#include <QLabel>
#include <QLine>
#include <QFrame>
#include <QPushButton>
#include <QGraphicsProxyWidget>
#include <QApplication>

// custom includes
#include "TfTransformRepSelectionWidget.h"

/*---------------------------------- public: -----------------------------{{{-*/
TfTransformWidget::TfTransformWidget(bool p_hasAbsolute, QWidget* p_parent)
  :QFrame(p_parent),
   m_hasAbsolute(p_hasAbsolute),
   m_broadcastCount(0),
   m_proxy(NULL)
{
  setFrameStyle(QFrame::Box);

  m_tf = new tf2::Transform();
  m_absoluteTf = new tf2::Transform();
  m_tfBroadcaster = new tf2_ros::TransformBroadcaster();

  m_tf->setOrigin(tf2::Vector3(0.0, 2.0, 0.0));
  tf2::Quaternion quat;
  quat.setEuler(0, 0, 0);
  m_tf->setRotation(quat);

  setFixedWidth(600);

  createLayout();
}

geometry_msgs::TransformStamped
TfTransformWidget::toTransformStamped(const tf2::Transform& p_tf, const std::string& p_tfParent, const std::string p_tfName, int p_seq)
{
  geometry_msgs::TransformStamped tfStampedMsg;
  tfStampedMsg.header.seq = p_seq;
  tfStampedMsg.header.stamp = ros::Time::now();
  tfStampedMsg.header.frame_id = p_tfParent;

  // tf2 sucks
  tfStampedMsg.transform.translation.x = p_tf.getOrigin().getX();
  tfStampedMsg.transform.translation.y = p_tf.getOrigin().getY();
  tfStampedMsg.transform.translation.z = p_tf.getOrigin().getZ();
  tfStampedMsg.transform.rotation.x = p_tf.getRotation().getX();
  tfStampedMsg.transform.rotation.y = p_tf.getRotation().getY();
  tfStampedMsg.transform.rotation.z = p_tf.getRotation().getZ();
  tfStampedMsg.transform.rotation.w = p_tf.getRotation().getW();

  tfStampedMsg.child_frame_id = p_tfName;

  return tfStampedMsg;
}

void
TfTransformWidget::setProxy(QGraphicsProxyWidget* p_proxy)
{
  m_proxy = p_proxy;
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
void
TfTransformWidget::broadcastTransform()
{
  if (m_tfName.empty() || m_tfParent.empty())
    return;

  m_tfBroadcaster->sendTransform(toTransformStamped(*m_tf, m_tfParent, m_tfName, m_broadcastCount++));
}

void
TfTransformWidget::toggleAbsolute(bool p_toggled)
{
  if (!m_hasAbsolute)
    return;

  if (p_toggled && m_topLayout->indexOf(m_absoluteRepSelectionWidget) == -1) {
    m_topLayout->addWidget(m_absoluteRepSelectionWidget, m_absoluteRepSelectionWidgetRow, m_absoluteRepSelectionWidgetColumn);
    m_absoluteRepSelectionWidget->show();
  } else {
    m_topLayout->removeWidget(m_absoluteRepSelectionWidget);
    m_absoluteRepSelectionWidget->hide();
  }

  // hack to resize Widget to size it would have if removed Widget would never have been there
  QApplication::processEvents();
  resize(0, 0);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfTransformWidget::createLayout()
{
  m_tfNameLabel = new QLabel("tfName:");
  m_tfNameEdit = new QLineEdit();
  connect(m_tfNameEdit, SIGNAL(editingFinished()), this, SLOT(setTfName()));

  m_tfNameLayout = new QHBoxLayout();
  m_tfNameLayout->addWidget(m_tfNameLabel);
  m_tfNameLayout->addWidget(m_tfNameEdit);

  m_relativeLabel = new QLabel("relative:");
  m_relativeRepSelectionWidget = new TfTransformRepSelectionWidget();
  m_relativeRepSelectionWidget->setTransform(m_tf);

  if (m_hasAbsolute) {
    m_horizontalLineFrame = new QFrame();
    m_horizontalLineFrame->setFrameShape(QFrame::HLine);

    m_absoluteButton = new QPushButton("absolute:");
    m_absoluteButton->setCheckable(true);
    m_absoluteRepSelectionWidget = new TfTransformRepSelectionWidget();
    m_absoluteRepSelectionWidget->setReadOnly(true);
    m_absoluteRepSelectionWidget->setTransform(m_absoluteTf);
    connect(m_absoluteButton, SIGNAL(toggled(bool)), this, SLOT(toggleAbsolute(bool)));
  }

  // populated from (1,1), child classes can easily insert something on all side
  m_topLayout = new QGridLayout();
  m_topLayout->addLayout(m_tfNameLayout, m_topLayout->rowCount(), 1);
  //m_topLayout->addWidget(m_relativeLabel, m_topLayout->rowCount(), 1);
  m_topLayout->addWidget(m_relativeRepSelectionWidget, m_topLayout->rowCount(), 1);
  if (m_hasAbsolute) {
    m_topLayout->addWidget(m_horizontalLineFrame, m_topLayout->rowCount(), 1);
    m_topLayout->addWidget(m_absoluteButton, m_topLayout->rowCount(), 1);
    m_absoluteRepSelectionWidgetRow = m_topLayout->rowCount();
    m_absoluteRepSelectionWidgetColumn = 1;
  }
  m_topLayout->setColumnStretch(1, 1);
  setLayout(m_topLayout);
  resize(0, 0);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
void
TfTransformWidget::setTfName()
{
  m_tfName = m_tfNameEdit->text().toStdString();
}

void
TfTransformWidget::setTfParent(const QString& p_parent)
{
  m_tfParent = p_parent.toStdString();
}
/*------------------------------------------------------------------------}}}-*/
