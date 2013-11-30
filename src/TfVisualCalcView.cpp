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

#include <yaml-cpp/yaml.h>

// custom includes

/*---------------------------------- public: -----------------------------{{{-*/
TfVisualCalcView::TfVisualCalcView(QWidget* p_parent)
  :QGraphicsView(p_parent),
   m_broadcastCount(0)
{
  m_nodeHandle = new ros::NodeHandle("tf_visual_calc_view");
  m_worldMapTf = new tf2::Transform(tf2::Quaternion(0, 0, 0, 1));
  m_tfBroadcaster = new tf2_ros::TransformBroadcaster();

  createScene();
  createContextMenu();
  setupBroadcastTimer();
}

TfTransformGraphicsWidget*
TfVisualCalcView::addTfWidget()
{
  return dynamic_cast<TfTransformGraphicsWidget*>(addTfWidgetInternal("")->widget());
}

TfTransformGraphicsWidget*
TfVisualCalcView::addTfWidget(const std::string& p_tfName)
{
  TfTransformGraphicsWidget* newWidget = addTfWidget();
  newWidget->setTfName(p_tfName);
  return newWidget;
}

void
TfVisualCalcView::deleteTfWidget(TfTransformGraphicsWidget* p_widget)
{
  if (std::find(m_toDeleteWidgets.begin(), m_toDeleteWidgets.end(), p_widget) == m_toDeleteWidgets.end()) {
    m_toDeleteWidgets.push_back(p_widget);
  }
}

ros::NodeHandle*
TfVisualCalcView::nodeHandle()
{
  return m_nodeHandle;
}

std::string
TfVisualCalcView::toYAMLString()
{
  YAML::Emitter out;
  out << YAML::BeginMap;
    out << YAML::Key << "root" << YAML::Value << m_rootTfWidget->tfName();
    out << YAML::Key << "transforms";
    out << YAML::Value << YAML::BeginMap;
      toYAML(out, m_rootTfWidget);
    out << YAML::EndMap;
  out << YAML::EndMap;


  return out.c_str();
}

void
TfVisualCalcView::fromYAMLString(const std::string& p_string)
{
  std::stringstream sstream(p_string);
  YAML::Parser parser(sstream);
  YAML::Node in;
  parser.GetNextDocument(in);

  if (in.FindValue("root")) {
    std::string rootTfName;
    in["root"] >> rootTfName;
    m_rootTfWidget->setTfName(rootTfName);
    m_rootTfWidget->fromYAML(in["transforms"]);
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
TfVisualCalcView::broadcastTransforms()
{
  if (!m_toDeleteWidgets.empty()) {
    for (int idx = 0; idx < m_toDeleteWidgets.size(); idx++) {
      QWidget* widget = m_toDeleteWidgets[idx];
      TfTransformGraphicsWidget* tfWidget = dynamic_cast<TfTransformGraphicsWidget*>(widget);
      tfWidget->parent()->deleteChild(tfWidget);
      delete widget;
    }
    m_toDeleteWidgets.clear();
    updateScene();
  }

  m_tfBroadcaster->sendTransform(TfTransformWidget::toTransformStamped(*m_worldMapTf, "/world", "/map", m_broadcastCount++));
  m_rootTfWidget->broadcastTransform();
}

void
TfVisualCalcView::updateScene()
{
  drawTree(m_rootTfWidget, 0, scene()->sceneRect().height() - m_worldLabel->height());
}

void
TfVisualCalcView::removeAll()
{
  m_rootTfWidget->deleteSubtree();
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

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfVisualCalcView::createScene()
{
  setScene(new QGraphicsScene(QRect(0, 0, 640, 480), this));
  scene()->setSceneRect(-2000, 0, 4000, 4000);

  m_worldLabel = new QLabel("/world");
  m_worldLabelProxy = scene()->addWidget(m_worldLabel);
  m_worldLabelProxy->setPos(-m_worldLabel->width()/2, scene()->sceneRect().height() - m_worldLabel->height()/2);

  QGraphicsProxyWidget* rootTfProxy = addTfWidgetInternal("/world", false);
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
TfVisualCalcView::addTfWidgetInternal(const std::string& p_tfName, bool p_hasAbsolute)
{
  TfTransformGraphicsWidget* newTfWidget = new RvizTfTransformGraphicsWidget(this, p_hasAbsolute);

  if (!p_tfName.empty()) {
    newTfWidget->setTfParentName(QString::fromStdString(p_tfName));
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

  // TODO init from tf2 (tf2_msgs/FrameGraph; ROS Service)

  m_contextMenu = new QMenu(this);
  m_contextMenu->addAction(m_removeAllAction);
}

void
TfVisualCalcView::drawTree(TfTransformGraphicsWidget* p_node, int p_x, int p_y)
{
  //printf("drawTree(%s, %d, %d)\n", p_node->tfName().c_str(), p_x, p_y);
  int nodeWidth = p_node->width();
  int nodeHeight = p_node->height();
  QGraphicsProxyWidget* proxy = p_node->proxy();
  proxy->setPos(p_x - nodeWidth/2, p_y - nodeHeight);

  int childrenCount = p_node->children().size();
  for (unsigned childIdx = 0; childIdx < childrenCount; childIdx++) {
    int dx = (childIdx - childrenCount/2) * nodeWidth;
    if (childrenCount % 2 == 0) {
      dx += nodeWidth/2;
    }
    int x = p_x + dx;
    int y = p_y - nodeHeight;
    drawTree(p_node->children()[childIdx], x, y);
  }
}

void
TfVisualCalcView::toYAML(YAML::Emitter& p_out, TfTransformGraphicsWidget* p_node)
{
  p_node->toYAML(p_out);
  for (unsigned childIdx = 0; childIdx < p_node->children().size(); childIdx++) {
    toYAML(p_out, p_node->children()[childIdx]);
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
