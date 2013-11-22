#ifndef _TF_TRANSFORM_GRAPHICS_WIDGET_H_
#define _TF_TRANSFORM_GRAPHICS_WIDGET_H_

// system includes
#include <vector>

// library includes

// custom includes
#include "TfTransformWidget.h"

// forward declarations
class QMenu;
class QAction;
class TfVisualCalcView;


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
    TfTransformGraphicsWidget(bool p_hasAbsolute = true, QWidget* p_parent = 0);

    // overwritten methods

    // methods
    void addChild(TfTransformGraphicsWidget* p_newChild);
    void setProxy(QGraphicsProxyWidget* p_proxy);
    void setView(TfVisualCalcView* p_view);
    QGraphicsProxyWidget* proxy();
    const std::vector<TfTransformGraphicsWidget*>& children();

    // variables


  public Q_SLOTS:
    virtual void broadcastTransform();
    virtual void toggleAbsolute(bool p_toggled);


  Q_SIGNALS:


  protected:
    // methods
    virtual void contextMenuEvent(QContextMenuEvent* p_event);

    // variables
    TfTransformGraphicsWidget* m_parent;
    std::vector<TfTransformGraphicsWidget*> m_children;


  protected Q_SLOTS:
    virtual void setTfName();


  private:
    // methods
    void extendLayout();
    void createContextMenu();

    // variables
    QLabel* m_childLabel;
    QLabel* m_parentLabel;

    QAction* m_createChildAction;
    QMenu* m_contextMenu;

    QGraphicsProxyWidget* m_proxy;
    TfVisualCalcView* m_view;

  private Q_SLOTS:
    void createChildWidget();


};

#endif // _TF_TRANSFORM_GRAPHICS_WIDGET_H_
