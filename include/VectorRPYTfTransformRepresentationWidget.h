#ifndef _VECTOR_RPY_TF_TRANSFORM_REPRESENTATION_WIDGET_H_
#define _VECTOR_RPY_TF_TRANSFORM_REPRESENTATION_WIDGET_H_

// system includes

// library includes

// custom includes
#include "TfTransformRepresentationWidget.h"
#include "VectorRPYGraphicWidget.h"

// forward declarations



class VectorRPYTfTransformRepresentationWidget
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
    VectorRPYTfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables


  public Q_SLOTS:
    virtual void setReadOnly(bool);
    virtual void updateDisplay();


  Q_SIGNALS:


  protected:


  protected Q_SLOTS:
    void updateTransform();


  private:
    // methods
    void createGraphicFrame();

    // variables
    VectorRPYGraphicWidget* m_graphicWidget;


  private Q_SLOTS:


};

#endif // _VECTOR_RPY_TF_TRANSFORM_REPRESENTATION_WIDGET_H_
