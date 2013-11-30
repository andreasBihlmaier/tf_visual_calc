#include "RvizTfTransformGraphicsWidget.h"

// system includes

// library includes
#include <visualization_msgs/Marker.h>

#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QStyle>
#include <QApplication>
#include <QFileDialog>

// custom includes
#include "TfVisualCalcView.h"


/*---------------------------------- public: -----------------------------{{{-*/
RvizTfTransformGraphicsWidget::RvizTfTransformGraphicsWidget(TfVisualCalcView* p_view, bool p_hasAbsolute, QWidget* p_parent)
  :TfTransformGraphicsWidget(p_view, p_hasAbsolute, p_parent)
{
  m_nodeHandle = m_view->nodeHandle();

  extendLayout();
  createMarkerPublisher();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
RvizTfTransformGraphicsWidget::extendLayout()
{
  m_markerLabel = new QLabel("Marker");
  m_markerEdit = new QLineEdit();
  connect(m_markerEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));
  m_markerFileButton = new QPushButton(QApplication::style()->standardIcon(QStyle::SP_FileIcon), tr("&Mesh"));
  connect(m_markerFileButton, SIGNAL(pressed()), this, SLOT(markerDialog()));

  m_markerScaleLabel = new QLabel("Scale");
  m_markerScaleXEdit = new QLineEdit("1.0");
  m_markerScaleXEdit->setFixedWidth(40);
  connect(m_markerScaleXEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));
  m_markerScaleYEdit = new QLineEdit("1.0");
  m_markerScaleYEdit->setFixedWidth(40);
  connect(m_markerScaleYEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));
  m_markerScaleZEdit = new QLineEdit("1.0");
  m_markerScaleZEdit->setFixedWidth(40);
  connect(m_markerScaleZEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));

  m_markerLayout = new QHBoxLayout();
  m_markerLayout->addWidget(m_markerLabel);
  m_markerLayout->addWidget(m_markerFileButton);
  m_markerLayout->addWidget(m_markerEdit);
  m_markerLayout->addWidget(m_markerScaleLabel);
  m_markerLayout->addWidget(m_markerScaleXEdit);
  m_markerLayout->addWidget(m_markerScaleYEdit);
  m_markerLayout->addWidget(m_markerScaleZEdit);

  m_topLayout->addLayout(m_markerLayout, m_topLayout->rowCount() + 1, 1);
  updateParentLabel();
}

void
RvizTfTransformGraphicsWidget::createMarkerPublisher()
{
  m_markerPublisher = new ros::Publisher(m_nodeHandle->advertise<visualization_msgs::Marker>("/visualization_marker", 0));
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
void
RvizTfTransformGraphicsWidget::markerDialog()
{
  QString markerFileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open Mesh as Marker"),
                                                        QString(),
                                                        tr("Meshes (*.dae *.stl *.mesh)"));
  if (!markerFileName.isEmpty()) {
    m_markerEdit->setText(markerFileName);
    updateMarker();
  }
}

void
RvizTfTransformGraphicsWidget::updateMarker()
{
  visualization_msgs::Marker markerMsg;
  markerMsg.header.frame_id = m_tfName;
  markerMsg.header.stamp = ros::Time::now();
  markerMsg.frame_locked = true;
  markerMsg.ns = m_tfName;
  markerMsg.id = 0;
  markerMsg.type = visualization_msgs::Marker::MESH_RESOURCE;
  markerMsg.action = visualization_msgs::Marker::ADD;
  markerMsg.pose.position.x = 0.0;
  markerMsg.pose.position.y = 0.0;
  markerMsg.pose.position.z = 0.0;
  markerMsg.pose.orientation.x = 0.0;
  markerMsg.pose.orientation.y = 0.0;
  markerMsg.pose.orientation.z = 0.0;
  markerMsg.pose.orientation.w = 1.0;
  markerMsg.scale.x = m_markerScaleXEdit->text().toDouble();
  markerMsg.scale.y = m_markerScaleYEdit->text().toDouble();
  markerMsg.scale.z = m_markerScaleZEdit->text().toDouble();
  markerMsg.mesh_use_embedded_materials = true;
  markerMsg.color.a = 0.0;
  markerMsg.color.r = 0.0;
  markerMsg.color.g = 0.0;
  markerMsg.color.b = 0.0;
  markerMsg.mesh_resource = m_markerEdit->text().toStdString();

  m_markerPublisher->publish(markerMsg);
}
/*------------------------------------------------------------------------}}}-*/
