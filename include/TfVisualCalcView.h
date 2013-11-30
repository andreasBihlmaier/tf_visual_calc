#ifndef _TF_VISUAL_CALC_VIEW_H_
#define _TF_VISUAL_CALC_VIEW_H_

// system includes

// library includes
#include <QGraphicsView>

// custom includes
#include "RvizTfTransformGraphicsWidget.h"


// forward declarations
class QLabel;
class QTimer;
class QPushButton;
class QMenu;
class QAction;

namespace YAML {
  class Emitter;
}


class TfVisualCalcView
  :public QGraphicsView
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions


    // constructors
    TfVisualCalcView(QWidget* p_parent = 0);

    // overwritten methods

    // methods
    TfTransformGraphicsWidget* addTfWidget();
    void deleteTfWidget(TfTransformGraphicsWidget* p_widget);
    ros::NodeHandle* nodeHandle();
    std::string toYAMLString();
    void fromYAMLString(const std::string& p_string);

    // variables


  public Q_SLOTS:
    void broadcastTransforms();
    void updateScene();
    void removeAll();


  Q_SIGNALS:


  protected:
    // methods
    virtual void contextMenuEvent(QContextMenuEvent* p_event);

    // variables
    tf2::Transform* m_worldMapTf;
    tf2_ros::TransformBroadcaster* m_tfBroadcaster;
    unsigned m_broadcastCount;


  protected Q_SLOTS:


  private:
    // methods
    void createScene();
    void createContextMenu();
    void setupBroadcastTimer();
    QGraphicsProxyWidget* addTfWidget(const std::string& p_tfName, bool p_hasAbsolute = true);
    void drawTree(TfTransformGraphicsWidget* p_node, int p_x, int p_y);
    void toYAML(YAML::Emitter& p_out, TfTransformGraphicsWidget* p_node);
    void fromYAML(const YAML::Node& p_in, TfTransformGraphicsWidget* p_node);

    // variables
    ros::NodeHandle* m_nodeHandle;
    QLabel* m_worldLabel;
    QGraphicsProxyWidget* m_worldLabelProxy;
    QTimer* m_broadcastTimer;
    TfTransformGraphicsWidget* m_rootTfWidget;

    QAction* m_removeAllAction;
    QMenu* m_contextMenu;

    std::vector<QWidget*> m_toDeleteWidgets;


  private Q_SLOTS:
    void adjustSize(QGraphicsProxyWidget* p_proxy);


};

#endif // _TF_VISUAL_CALC_VIEW_H_
