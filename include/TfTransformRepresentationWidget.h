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
#include <QString>

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
    static QString number(double p_num);


    // constructors
    TfTransformRepresentationWidget(tf2::Transform* p_tf, QWidget* p_parent = 0);

    // overwritten methods

    // methods
    void setTransform(tf2::Transform* p_tf);

    // variables


  public Q_SLOTS:
    virtual void setReadOnly(bool);
    virtual void updateDisplay() = 0;


  Q_SIGNALS:


  protected:
    // methods
    void setText(const QString& p_text);

    // variables
    tf2::Transform* m_tf;
    QPlainTextEdit* m_textEdit;
    QHBoxLayout* m_topLayout;


  protected Q_SLOTS:
    virtual void updateTransformFromText() = 0;


  private:
    // methods
    void createLayout();

    // variables


  private Q_SLOTS:


};

#endif // _TF_TRANSFORM_REPRESENTATION_WIDGET_H_
