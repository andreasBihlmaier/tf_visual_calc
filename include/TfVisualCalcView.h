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

    // variables


  public slots:
    void broadcastTransforms();


  signals:


  protected:
    // methods

    // variables
    tf2::Transform* m_worldMapTf;
    tf2_ros::TransformBroadcaster* m_tfBroadcaster;
    unsigned m_broadcastCount;


  protected slots:


  private:
    // methods
    void createScene();
    void setupBroadcastTimer();

    // variables
    QLabel* m_worldLabel;
    QTimer* m_broadcastTimer;
    TfTransformGraphicsWidget* m_rootTfWidget;


  private slots:


};

#endif // _TF_VISUAL_CALC_VIEW_H_
