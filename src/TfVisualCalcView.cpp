#include "TfVisualCalcView.h"

// system includes
#include <iostream>

// library includes
#include <QLabel>
#include <QGraphicsScene>
#include <QGraphicsProxyWidget>
#include <QGraphicsSceneContextMenuEvent>
#include <QTimer>
#include <QPushButton>
#include <QAction>
#include <QMenu>

// custom includes

/*---------------------------------- public: -----------------------------{{{-*/
TfVisualCalcView::TfVisualCalcView(QWidget* p_parent)
  :QGraphicsView(p_parent),
   m_broadcastCount(0)
{
  m_worldMapTf = new tf2::Transform();
  m_tfBroadcaster = new tf2_ros::TransformBroadcaster();

  createScene();
  createContextMenu();
  setupBroadcastTimer();
}

TfTransformGraphicsWidget*
TfVisualCalcView::addTfWidget()
{
  return dynamic_cast<TfTransformGraphicsWidget*>(addTfWidget("")->widget());
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
TfVisualCalcView::updateScene()
{
  drawTree(m_rootTfWidget, 0, scene()->sceneRect().height() - m_worldLabel->height());
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
void
TfVisualCalcView::contextMenuEvent(QContextMenuEvent* p_event)
{
  QGraphicsSceneContextMenuEvent* graphicsSceneEvent = dynamic_cast<QGraphicsSceneContextMenuEvent*>(p_event);
  if (items(p_event->pos()).empty()) {
    m_contextMenu->exec(p_event->globalPos());
  } else {
    QGraphicsView::contextMenuEvent(p_event);
  }
}
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

  QGraphicsProxyWidget* rootTfProxy = addTfWidget("/world", false);
  m_rootTfWidget = dynamic_cast<TfTransformGraphicsWidget*>(rootTfProxy->widget());
  updateScene();

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
TfVisualCalcView::addTfWidget(const std::string& p_tfName, bool p_hasAbsolute)
{
  TfTransformGraphicsWidget* newTfWidget = new TfTransformGraphicsWidget(p_hasAbsolute);
  newTfWidget->setView(this);

  if (!p_tfName.empty()) {
    newTfWidget->setTfParent(QString::fromStdString(p_tfName));
  }

  QGraphicsProxyWidget* newTfProxy = scene()->addWidget(newTfWidget);
  newTfWidget->setProxy(newTfProxy);

  return newTfProxy;
}

void
TfVisualCalcView::createContextMenu()
{
  m_removeAllAction = new QAction(tr("&Remove all"), this);
  connect(m_removeAllAction, SIGNAL(triggered()), this, SLOT(removeAll()));

  m_contextMenu = new QMenu(this);
  m_contextMenu->addAction(m_removeAllAction);
}

void
TfVisualCalcView::drawTree(TfTransformGraphicsWidget* p_node, int p_x, int p_y)
{
  QGraphicsProxyWidget* proxy = p_node->proxy();
  proxy->setPos(-p_node->width()/2, p_y - p_node->height());

  for (unsigned childIdx = 0; childIdx < p_node->children().size(); childIdx++) {
    drawTree(p_node->children()[childIdx], p_x, p_y - p_node->height());
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
