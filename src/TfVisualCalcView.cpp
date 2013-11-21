#include "TfVisualCalcView.h"

// system includes
#include <iostream>

// library includes
#include <QLabel>
#include <QGraphicsScene>
#include <QGraphicsProxyWidget>
#include <QTimer>
#include <QPushButton>

// custom includes

/*---------------------------------- public: -----------------------------{{{-*/
TfVisualCalcView::TfVisualCalcView(QWidget* p_parent)
  :QGraphicsView(p_parent),
   m_broadcastCount(0)
{
  m_worldMapTf = new tf2::Transform();
  m_tfBroadcaster = new tf2_ros::TransformBroadcaster();

  createScene();
  setupBroadcastTimer();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
void
TfVisualCalcView::broadcastTransforms()
{
  m_tfBroadcaster->sendTransform(TfTransformWidget::toTransformStamped(*m_worldMapTf, "/world", "/map", m_broadcastCount++));
  m_rootTfWidget->broadcastTransform();
}

void
TfVisualCalcView::addTransformWidget()
{
  // TODO
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
void
TfVisualCalcView::resizeEvent(QResizeEvent* p_event)
{
  QGraphicsView::resizeEvent(p_event);
  // TODO
  scene()->setSceneRect(rect());

  std::cout << m_worldLabel->pos().x() << std::endl;
  m_worldLabelProxy->setPos(width()/2 - m_worldLabel->width()/2, height() - m_worldLabel->height()/2);
  m_addButtonProxy->setPos(width() - m_addButton->width()/2, height() - m_addButton->height()/2);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected slots: ------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfVisualCalcView::createScene()
{
  setScene(new QGraphicsScene(QRect(0, 0, 640, 480), this));

  m_worldLabel = new QLabel("/world");
  m_worldLabelProxy = scene()->addWidget(m_worldLabel);

  m_rootTfWidget = new TfTransformGraphicsWidget(this);
  m_rootTfWidget->setTfParent("/world");

  m_addButton = new QPushButton("Add Tf");
  connect(m_addButton, SIGNAL(pressed()), this, SLOT(addTransformWidget()));
  m_addButtonProxy = scene()->addWidget(m_addButton);
}

void
TfVisualCalcView::setupBroadcastTimer()
{
  m_broadcastTimer = new QTimer(this);
  m_broadcastTimer->setInterval(100);
  connect(m_broadcastTimer, SIGNAL(timeout()), this, SLOT(broadcastTransforms()));
  m_broadcastTimer->start();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
