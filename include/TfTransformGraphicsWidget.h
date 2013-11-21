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

    // variables


  public slots:
    virtual void broadcastTransform();


  signals:


  protected:
    // methods
    virtual void contextMenuEvent(QContextMenuEvent* p_event);

    // variables
    TfTransformGraphicsWidget* m_parent;
    std::vector<TfTransformGraphicsWidget*> m_children;


  protected slots:


  private:
    // methods
    void extendLayout();
    void createContextMenu();

    // variables
    QLabel* m_childLabel;
    QLabel* m_parentLabel;

    QAction* m_addChildAction;
    QMenu* m_contextMenu;

  private slots:


};

#endif // _TF_TRANSFORM_GRAPHICS_WIDGET_H_
