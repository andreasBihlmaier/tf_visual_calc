#ifndef _VECTOR_QUATERNION_GRAPHIC_WIDGET_H_
#define _VECTOR_QUATERNION_GRAPHIC_WIDGET_H_

// system includes

// library includes
#include <QWidget>
#include <QLineEdit>
#include <QLabel>

// custom includes


// forward declarations


class VectorQuaternionGraphicWidget
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
    VectorQuaternionGraphicWidget(QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables
    QLabel* m_translationLabel;
    QLabel* m_rotationLabel;
    QLabel* m_wLabel;
    QLineEdit* m_xEdit;
    QLineEdit* m_yEdit;
    QLineEdit* m_zEdit;
    QLineEdit* m_qxEdit;
    QLineEdit* m_qyEdit;
    QLineEdit* m_qzEdit;
    QLineEdit* m_qwEdit;


  public Q_SLOTS:


  Q_SIGNALS:


  protected:
    void paintEvent(QPaintEvent* p_event);


  private:
    // methods
    void createChildWidgets();

    // variables

  private Q_SLOTS:


};

#endif // _VECTOR_QUATERNION_GRAPHIC_WIDGET_H_
