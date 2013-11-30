#ifndef _RVIZ_TF_TRANSFORM_GRAPHICS_WIDGET_H_
#define _RVIZ_TF_TRANSFORM_GRAPHICS_WIDGET_H_

// system includes

// library includes

// custom includes
#include "TfTransformGraphicsWidget.h"

// forward declarations



class RvizTfTransformGraphicsWidget
  :public TfTransformGraphicsWidget
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions


    // constructors
    RvizTfTransformGraphicsWidget(TfVisualCalcView* p_view, bool p_hasAbsolute = true, QWidget* p_parent = 0);

    // overwritten methods
    virtual void toYAML(YAML::Emitter& p_out);
    virtual void fromYAML(const YAML::Node& p_in);

    // methods

    // variables


  public Q_SLOTS:
    virtual void broadcastTransform();


  Q_SIGNALS:


  protected:
    // methods

    // variables


  protected Q_SLOTS:


  private:
    // methods
    void extendLayout();
    void setMarkerScale(double p_x, double p_y, double p_z);

    // variables
    tf2::Transform m_lastTf;

    QLabel* m_markerLabel;
    QLineEdit* m_markerEdit;
    QHBoxLayout* m_markerLayout;
    QPushButton* m_markerFileButton;

    QLabel* m_markerScaleLabel;
    QLineEdit* m_markerScaleXEdit;
    QLineEdit* m_markerScaleYEdit;
    QLineEdit* m_markerScaleZEdit;

    ros::NodeHandle* m_nodeHandle;
    ros::Publisher* m_markerPublisher;


  private Q_SLOTS:
    void markerDialog();
    void updateMarker();
    void createMarkerPublisher();


};

#endif // _RVIZ_TF_TRANSFORM_GRAPHICS_WIDGET_H_
