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
TfVisualCalcView::addTfWidget()
{
  // TODO
  // addTfWidget("");
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected slots: ------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfVisualCalcView::createScene()
{
  setScene(new QGraphicsScene(QRect(0, 0, 640, 480), this));
  scene()->setSceneRect(-1000, 0, 2000, 1000);

  m_worldLabel = new QLabel("/world");
  m_worldLabelProxy = scene()->addWidget(m_worldLabel);
  m_worldLabelProxy->setPos(-m_worldLabel->width()/2, scene()->sceneRect().height() - m_worldLabel->height()/2);

  QGraphicsProxyWidget* rootTfProxy = addTfWidget("/world");
  m_rootTfWidget = (TfTransformGraphicsWidget*)rootTfProxy->widget();
  rootTfProxy->setPos(-m_rootTfWidget->width()/2, scene()->sceneRect().height() - m_worldLabel->height() - m_rootTfWidget->height());
  //TODO m_rootTfWidget->moveable(false);

  centerOn(0, scene()->sceneRect().height());
}

void
TfVisualCalcView::setupBroadcastTimer()
{
  m_broadcastTimer = new QTimer(this);
  m_broadcastTimer->setInterval(100);
  connect(m_broadcastTimer, SIGNAL(timeout()), this, SLOT(broadcastTransforms()));
  m_broadcastTimer->start();
}

QGraphicsProxyWidget*
TfVisualCalcView::addTfWidget(const std::string& p_tfName)
{
  TfTransformGraphicsWidget* newTfWidget = new TfTransformGraphicsWidget();

  if (!p_tfName.empty()) {
    newTfWidget->setTfParent(QString::fromStdString(p_tfName));
  }

  QGraphicsProxyWidget* newTfProxy = scene()->addWidget(newTfWidget);
  newTfWidget->setProxy(newTfProxy);
  newTfProxy->setPos(mapToScene(width()/2, height()/2));

  return newTfProxy;
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
