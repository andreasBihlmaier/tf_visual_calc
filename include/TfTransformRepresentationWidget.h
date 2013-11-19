#ifndef _TF_TRANSFORM_REPRESENTATION_WIDGET_H_
#define _TF_TRANSFORM_REPRESENTATION_WIDGET_H_

// system includes

// library includes
#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

// custom includes


// forward declarations

namespace tf2 {
  class Transform;
}


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
    TfTransformRepresentationWidget(QWidget* p_parent, tf2::Transform* p_tf);

    // overwritten methods

    // methods

    // variables


  public slots:
    virtual void setReadOnly(bool);


  signals:


  protected:
    // methods
    tf2::Transform* m_tf;

    // variables
    QLineEdit* m_textEdit;
    QVBoxLayout* m_topLayout;


  private:
    // methods
    void createLayout();

    // variables


  private slots:


};

#endif // _TF_TRANSFORM_REPRESENTATION_WIDGET_H_
