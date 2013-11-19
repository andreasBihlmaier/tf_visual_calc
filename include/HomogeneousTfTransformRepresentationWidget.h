#ifndef _HOMOGENEOUS_TF_TRANSFORM_REPRESENTATION_WIDGET_H_
#define _HOMOGENEOUS_TF_TRANSFORM_REPRESENTATION_WIDGET_H_

// system includes

// library includes

// custom includes
#include "TfTransformRepresentationWidget.h"

// forward declarations



class HomogeneousTfTransformRepresentationWidget
  :public TfTransformRepresentationWidget
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions


    // constructors
    HomogeneousTfTransformRepresentationWidget(QWidget* p_parent, tf2::Transform* p_tf);

    // overwritten methods

    // methods

    // variables


  public slots:
    virtual void setReadOnly(bool);


  signals:


  private:
    // methods

    // variables


  private slots:


};

#endif // _HOMOGENEOUS_TF_TRANSFORM_REPRESENTATION_WIDGET_H_
