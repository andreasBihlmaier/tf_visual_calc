#ifndef _TF_VISUAL_CALC_VIEW_H_
#define _TF_VISUAL_CALC_VIEW_H_

// system includes

// library includes
#include <QGraphicsView>

// custom includes
#include "TfTransformGraphicsWidget.h"


// forward declarations
class QLabel;
class QTimer;
class QPushButton;
class QMenu;
class QAction;


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

    // variables


  public slots:
    void broadcastTransforms();
    void updateScene();


  signals:


  protected:
    // methods
    virtual void contextMenuEvent(QContextMenuEvent* p_event);

    // variables
    tf2::Transform* m_worldMapTf;
    tf2_ros::TransformBroadcaster* m_tfBroadcaster;
    unsigned m_broadcastCount;


  protected slots:


  private:
    // methods
    void createScene();
    void createContextMenu();
    void setupBroadcastTimer();
    QGraphicsProxyWidget* addTfWidget(const std::string& p_tfName, bool p_hasAbsolute = true);
    void drawTree(TfTransformGraphicsWidget* p_node, int p_x, int p_y);

    // variables
    QLabel* m_worldLabel;
    QGraphicsProxyWidget* m_worldLabelProxy;
    QTimer* m_broadcastTimer;
    TfTransformGraphicsWidget* m_rootTfWidget;

    QAction* m_removeAllAction;
    QMenu* m_contextMenu;


  private slots:
    void adjustSize(QGraphicsProxyWidget* p_proxy);


};

#endif // _TF_VISUAL_CALC_VIEW_H_
