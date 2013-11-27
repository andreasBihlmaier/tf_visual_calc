#ifndef _DH_GRAPHIC_WIDGET_H_
#define _DH_GRAPHIC_WIDGET_H_

// system includes

// library includes
#include <QWidget>
#include <QLineEdit>
#include <QImage>

// custom includes


// forward declarations


class DHGraphicWidget
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
    DHGraphicWidget(QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables
    QLineEdit* m_dEdit;
    QLineEdit* m_aEdit;
    QLineEdit* m_thetaEdit;
    QLineEdit* m_alphaEdit;

    QImage* m_dhImage;


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

#endif // _DH_GRAPHIC_WIDGET_H_
