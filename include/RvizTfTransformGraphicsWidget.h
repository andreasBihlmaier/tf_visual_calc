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
    RvizTfTransformGraphicsWidget(bool p_hasAbsolute = true, QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables


  public Q_SLOTS:


  Q_SIGNALS:


  protected:
    // methods

    // variables


  protected Q_SLOTS:


  private:
    // methods
    void extendLayout();

    // variables
    QLabel* m_markerLabel;
    QLineEdit* m_markerEdit;
    QHBoxLayout* m_markerLayout;
    QPushButton* m_markerFileButton;

    QLabel* m_markerScaleLabel;
    QLineEdit* m_markerScaleXEdit;
    QLineEdit* m_markerScaleYEdit;
    QLineEdit* m_markerScaleZEdit;


  private Q_SLOTS:
    void markerDialog();
    void updateMarker();


};

#endif // _RVIZ_TF_TRANSFORM_GRAPHICS_WIDGET_H_
