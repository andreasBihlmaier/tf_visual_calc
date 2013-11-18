#ifndef _TF_TRANSFORM_WIDGET_H_
#define _TF_TRANSFORM_WIDGET_H_

// system includes

// library includes
#include <QWidget>

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <tf2_ros/transform_broadcaster.h>
# include <tf2/LinearMath/Transform.h>
#endif

// custom includes


// forward declarations



class TfTransformWidget
  :public QWidget
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions


    // constructors

    // overwritten methods

    // methods

    // variables


  public slots:


  signals:


  private:
    // methods

    // variables
    tf2::Transform m_tf;
    tf2_ros::TransformBroadcaster m_tfBroadcaster;


  private slots:


};

#endif // _TF_TRANSFORM_WIDGET_H_
