#ifndef _DH_TF_TRANSFORM_REPRESENTATION_WIDGET_H_
#define _DH_TF_TRANSFORM_REPRESENTATION_WIDGET_H_

// system includes

// library includes

// custom includes
#include "TfTransformRepresentationWidget.h"
#include "DHGraphicWidget.h"

// forward declarations



class DHTfTransformRepresentationWidget
  :public TfTransformRepresentationWidget
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions
    static tf2::Transform dh2Transform(double p_d, double p_theta, double p_a, double p_alpha);
    static bool isEqual(const tf2::Transform& p_tfA, const tf2::Transform& p_tfB);


    // constructors
    DHTfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables


  public Q_SLOTS:
    virtual void setReadOnly(bool);
    virtual void updateDisplay();


  Q_SIGNALS:


  protected:


  protected Q_SLOTS:
    void updateTransformFromGraphic();
    virtual void updateTransformFromText();


  private:
    // methods
    void createGraphicFrame();

    // variables
    DHGraphicWidget* m_graphicWidget;


  private Q_SLOTS:


};

#endif // _DH_TF_TRANSFORM_REPRESENTATION_WIDGET_H_
