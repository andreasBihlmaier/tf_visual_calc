#ifndef _TF_TRANSFORM_GRAPHICS_WIDGET_H_
#define _TF_TRANSFORM_GRAPHICS_WIDGET_H_

// system includes

// library includes

// custom includes
#include "TfTransformWidget.h"

// forward declarations



class TfTransformGraphicsWidget
  :public TfTransformWidget
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions


    // constructors
    TfTransformGraphicsWidget(QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables


  public slots:


  signals:


  protected:
    // methods

    // variables
    // TODO parent, children


  protected slots:


  private:
    // methods
    void extendLayout();

    // variables
    QLabel* m_childLabel;
    QLabel* m_parentLabel;


  private slots:


};

#endif // _TF_TRANSFORM_GRAPHICS_WIDGET_H_
