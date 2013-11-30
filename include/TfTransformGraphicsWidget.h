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

namespace YAML {
  class Emitter;
  class Node;
}


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
    TfTransformGraphicsWidget(TfVisualCalcView* p_view, bool p_hasAbsolute = true, QWidget* p_parent = 0);

    // overwritten methods

    // methods
    void addChild(TfTransformGraphicsWidget* p_newChild);
    void deleteChild(TfTransformGraphicsWidget* p_newChild);
    void setProxy(QGraphicsProxyWidget* p_proxy);
    void setView(TfVisualCalcView* p_view);
    QGraphicsProxyWidget* proxy();
    const std::vector<TfTransformGraphicsWidget*>& children();
    TfTransformGraphicsWidget* parent();
    void setTfParent(TfTransformGraphicsWidget* p_parent);
    virtual void toYAML(YAML::Emitter& p_out);
    virtual void fromYAML(const YAML::Node& p_in);

    // variables


  public Q_SLOTS:
    virtual void broadcastTransform();
    virtual void toggleAbsolute(bool p_toggled);
    virtual void updateSize();
    virtual void deleteSubtree();
    virtual void setTfName();
    void setTfName(const std::string& p_tfName);


  Q_SIGNALS:


  protected:
    // methods
    virtual void contextMenuEvent(QContextMenuEvent* p_event);
    void updateParentLabel();
    void toYAMLStart(YAML::Emitter& p_out);
    void toYAMLData(YAML::Emitter& p_out);
    void toYAMLEnd(YAML::Emitter& p_out);

    // variables
    TfTransformGraphicsWidget* m_parent;
    std::vector<TfTransformGraphicsWidget*> m_children;
    TfVisualCalcView* m_view;


  protected Q_SLOTS:


  private:
    // methods
    void extendLayout();
    void createContextMenu();

    // variables
    QLabel* m_childLabel;
    QLabel* m_parentLabel;

    QAction* m_createChildAction;
    QAction* m_deleteAction;
    QAction* m_deleteSubtreeAction;
    QMenu* m_contextMenu;

    QGraphicsProxyWidget* m_proxy;

  private Q_SLOTS:
    void createChildWidget();
    void deleteWidget();


};

#endif // _TF_TRANSFORM_GRAPHICS_WIDGET_H_
