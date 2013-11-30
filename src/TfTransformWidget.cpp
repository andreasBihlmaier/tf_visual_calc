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
   m_broadcastCount(0)
{
  setFrameStyle(QFrame::Box);

  m_tf = new tf2::Transform();
  m_absoluteTf = new tf2::Transform();

  m_tf->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion quat;
  quat.setEuler(0, 0, 0);
  m_tf->setRotation(quat);
  m_absoluteTf->setOrigin(tf2::Vector3(0, 0, 0));
  m_absoluteTf->setRotation(quat);

  m_tfBuffer = new tf2_ros::Buffer();
  m_tfBroadcaster = new tf2_ros::TransformBroadcaster();
  m_tfListener = new tf2_ros::TransformListener(*m_tfBuffer);

  setFixedWidth(600);

  createLayout();
}

// tf2 sucks
tf2::Transform
TfTransformWidget::toTransform(const geometry_msgs::TransformStamped& p_tfStamped)
{
  return tf2::Transform(tf2::Quaternion(p_tfStamped.transform.rotation.x,
                                        p_tfStamped.transform.rotation.y,
                                        p_tfStamped.transform.rotation.z,
                                        p_tfStamped.transform.rotation.w),
                        tf2::Vector3(p_tfStamped.transform.translation.x,
                                     p_tfStamped.transform.translation.y,
                                     p_tfStamped.transform.translation.z));
}

geometry_msgs::TransformStamped
TfTransformWidget::toTransformStamped(const tf2::Transform& p_tf, const std::string& p_tfParentName, const std::string p_tfName, int p_seq)
{
  geometry_msgs::TransformStamped tfStampedMsg;
  tfStampedMsg.header.seq = p_seq;
  tfStampedMsg.header.stamp = ros::Time::now();
  tfStampedMsg.header.frame_id = p_tfParentName;

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

const std::string&
TfTransformWidget::tfName()
{
  return m_tfName;
}

const tf2::Transform*
TfTransformWidget::tf()
{
  return m_tf;
}

int
TfTransformWidget::relativeRepresentation()
{
  return m_relativeRepSelectionWidget->representation();
}

bool
TfTransformWidget::hasAbsoluteRepresentation()
{
  return m_hasAbsolute;
}

int
TfTransformWidget::absoluteRepresentation()
{
  if (m_hasAbsolute) {
    return m_absoluteRepSelectionWidget->representation();
  } else {
    return -1;
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
TfTransformWidget::broadcastTransform()
{
  if (m_tfName.empty() || m_tfParentName.empty())
    return;

  m_tfBroadcaster->sendTransform(toTransformStamped(*m_tf, m_tfParentName, m_tfName, m_broadcastCount++));

  if (m_hasAbsolute) {
    std::string errorString;
    if (!m_tfBuffer->canTransform("world", m_tfName.substr(1), ros::Time(0), ros::Duration(0), &errorString)) {
      std::cout << "errorString=" << errorString << std::endl;
    } else {
      try {
        geometry_msgs::TransformStamped transformStamped = m_tfBuffer->lookupTransform("world", m_tfName.substr(1), ros::Time(0));
        *m_absoluteTf = toTransform(transformStamped);
        m_absoluteRepSelectionWidget->updateDisplay();
      } catch (tf2::LookupException) {
        std::cout << "tf2::LookupException" << std::endl;
      }
    }
  }
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

  updateSize();
}

void
TfTransformWidget::updateSize()
{
  // hack to resize Widget to size it would have if removed Widget would never have been there
  QApplication::processEvents();
  resize(0, 0);
}

void
TfTransformWidget::setRelativeRepresentation(int p_representation)
{
  m_relativeRepSelectionWidget->setRepresentation(p_representation);
}

void
TfTransformWidget::setAbsoluteRepresentation(int p_representation)
{
  m_absoluteRepSelectionWidget->setRepresentation(p_representation);
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
void
TfTransformWidget::setTfName(const std::string& p_tfName)
{
  m_tfNameEdit->setText(QString::fromStdString(p_tfName));
}

void
TfTransformWidget::setTf(const tf2::Transform& p_tf)
{
  *m_tf = p_tf;
  updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
void
TfTransformWidget::setTfName()
{
  m_tfName = m_tfNameEdit->text().toStdString();
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
  connect(m_relativeRepSelectionWidget, SIGNAL(sizeChanged()), this, SLOT(updateSize()));

  if (m_hasAbsolute) {
    m_horizontalLineFrame = new QFrame();
    m_horizontalLineFrame->setFrameShape(QFrame::HLine);

    m_absoluteButton = new QPushButton("absolute:");
    m_absoluteButton->setCheckable(true);
    connect(m_absoluteButton, SIGNAL(toggled(bool)), this, SLOT(toggleAbsolute(bool)));
    m_absoluteRepSelectionWidget = new TfTransformRepSelectionWidget();
    m_absoluteRepSelectionWidget->setReadOnly(true);
    m_absoluteRepSelectionWidget->setTransform(m_absoluteTf);
    connect(m_absoluteRepSelectionWidget, SIGNAL(sizeChanged()), this, SLOT(updateSize()));
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

void
TfTransformWidget::updateDisplay()
{
  m_relativeRepSelectionWidget->updateDisplay();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
void
TfTransformWidget::setTfParentName(const QString& p_parent)
{
  m_tfParentName = p_parent.toStdString();
}
/*------------------------------------------------------------------------}}}-*/
