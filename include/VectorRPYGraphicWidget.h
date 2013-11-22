#ifndef _VECTOR_RPY_GRAPHIC_WIDGET_H_
#define _VECTOR_RPY_GRAPHIC_WIDGET_H_

// system includes

// library includes
#include <QWidget>
#include <QLineEdit>

// custom includes


// forward declarations


class VectorRPYGraphicWidget
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
    VectorRPYGraphicWidget(QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables
    QLineEdit* m_xEdit;
    QLineEdit* m_yEdit;
    QLineEdit* m_zEdit;
    QLineEdit* m_rxEdit;
    QLineEdit* m_ryEdit;
    QLineEdit* m_rzEdit;


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

#endif // _VECTOR_RPY_GRAPHIC_WIDGET_H_
