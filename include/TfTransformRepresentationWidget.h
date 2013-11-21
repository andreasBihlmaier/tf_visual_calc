#ifndef _TF_TRANSFORM_REPRESENTATION_WIDGET_H_
#define _TF_TRANSFORM_REPRESENTATION_WIDGET_H_

// system includes

// library includes
#include <QWidget>
#include <QLabel>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

// custom includes


// forward declarations


class TfTransformRepresentationWidget
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
    TfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent = 0);

    // overwritten methods

    // methods
    void setTransform(tf2::Transform* p_tf);

    // variables


  public slots:
    virtual void setReadOnly(bool);
    virtual void updateDisplay() = 0;


  signals:


  protected:
    // methods
    tf2::Transform* m_tf;

    // variables
    QPlainTextEdit* m_textEdit;
    QHBoxLayout* m_topLayout;


  protected slots:


  private:
    // methods
    void createLayout();

    // variables


  private slots:


};

#endif // _TF_TRANSFORM_REPRESENTATION_WIDGET_H_
