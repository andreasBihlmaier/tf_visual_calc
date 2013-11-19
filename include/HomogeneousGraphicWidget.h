#ifndef _HOMOGENEOUS_GRAPHIC_WIDGET_H_
#define _HOMOGENEOUS_GRAPHIC_WIDGET_H_

// system includes

// library includes
#include <QWidget>

// custom includes


// forward declarations
class QLineEdit;


class HomogeneousGraphicWidget
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
    HomogeneousGraphicWidget(QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables
    QLineEdit* matrixEdits[4][4];


  public slots:


  signals:


  protected:
    void paintEvent(QPaintEvent* p_event);


  private:
    // methods
    void createChildWidgets();

    // variables


  private slots:


};

#endif // _HOMOGENEOUS_GRAPHIC_WIDGET_H_
