#include "TfTransformGraphicsWidget.h"

// system includes
#include <algorithm>

// library includes
#include <QLabel>
#include <QAction>
#include <QMenu>
#include <QContextMenuEvent>

// custom includes
#include "TfVisualCalcView.h"

/*---------------------------------- public: -----------------------------{{{-*/
TfTransformGraphicsWidget::TfTransformGraphicsWidget(TfVisualCalcView* p_view, bool p_hasAbsolute, QWidget* p_parent)
  :TfTransformWidget(p_hasAbsolute, p_parent),
   m_parent(NULL),
   m_proxy(NULL),
   m_view(p_view)
{
  createContextMenu();
  extendLayout();
}

void
TfTransformGraphicsWidget::addChild(TfTransformGraphicsWidget* p_newChild)
{
  if (std::find(m_children.begin(), m_children.end(), p_newChild) == m_children.end()) {
    m_children.push_back(p_newChild);
    p_newChild->setTfParent(this);
  }
}

void
TfTransformGraphicsWidget::deleteChild(TfTransformGraphicsWidget* p_child)
{
  std::cout << m_tfName << " deleteChild(" << p_child->tfName() << ")" << std::endl;
  std::vector<TfTransformGraphicsWidget*>::iterator childIter = std::find(m_children.begin(), m_children.end(), p_child);
  m_children.erase(childIter);
  for (std::vector<TfTransformGraphicsWidget*>::const_iterator grandchildIter = p_child->children().begin();
       grandchildIter != p_child->children().end();
       grandchildIter++) {
    std::cout << "Setting parent of " << (*grandchildIter)->tfName() << " to " << m_tfName << std::endl;
    m_children.push_back(*grandchildIter);
    (*grandchildIter)->setTfParent(this);
  }
}

void
TfTransformGraphicsWidget::setProxy(QGraphicsProxyWidget* p_proxy)
{
  m_proxy = p_proxy;
}

QGraphicsProxyWidget*
TfTransformGraphicsWidget::proxy()
{
  return m_proxy;
}

const std::vector<TfTransformGraphicsWidget*>&
TfTransformGraphicsWidget::children()
{
  return m_children;
}

void
TfTransformGraphicsWidget::setTfParent(TfTransformGraphicsWidget* p_parent)
{
  m_parent = p_parent;
  setTfParentName(QString::fromStdString(m_parent->tfName()));
}

TfTransformGraphicsWidget*
TfTransformGraphicsWidget::parent()
{
  return m_parent;
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
void
TfTransformGraphicsWidget::broadcastTransform()
{
  TfTransformWidget::broadcastTransform();

  for (std::vector<TfTransformGraphicsWidget*>::iterator childIter = m_children.begin();
       childIter != m_children.end();
       ++childIter) {
    (*childIter)->broadcastTransform();
  }
}

void
TfTransformGraphicsWidget::toggleAbsolute(bool p_toggled)
{
  TfTransformWidget::toggleAbsolute(p_toggled);
  m_view->updateScene();
}

void
TfTransformGraphicsWidget::updateSize()
{
  TfTransformWidget::updateSize();
  m_view->updateScene();
}

void
TfTransformGraphicsWidget::deleteSubtree()
{
  for (std::vector<TfTransformGraphicsWidget*>::iterator childIter = m_children.begin();
       childIter != m_children.end();
       ++childIter) {
    (*childIter)->deleteSubtree();
  }

  // second iteration because grandchildren become children when a child is deleted
  for (std::vector<TfTransformGraphicsWidget*>::iterator childIter = m_children.begin();
       childIter != m_children.end();
       ++childIter) {
    (*childIter)->deleteWidget();
  }
}
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
void
TfTransformGraphicsWidget::contextMenuEvent(QContextMenuEvent* p_event)
{
  m_contextMenu->exec(p_event->globalPos());
}

void
TfTransformGraphicsWidget::updateParentLabel()
{
  m_topLayout->removeWidget(m_parentLabel);
  m_topLayout->addWidget(m_parentLabel, m_topLayout->rowCount() - 1, 0, Qt::AlignBottom);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
void
TfTransformGraphicsWidget::setTfName()
{
  TfTransformWidget::setTfName();

  for (std::vector<TfTransformGraphicsWidget*>::iterator childIter = m_children.begin();
       childIter != m_children.end();
       ++childIter) {
    (*childIter)->setTfParent(this);
  }
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfTransformGraphicsWidget::extendLayout()
{
  QFont font;
  font.setPointSize(6);
  m_childLabel = new QLabel(QString(QChar(0x2191)) + "children");
  m_childLabel->setFont(font);
  m_parentLabel = new QLabel(QString(QChar(0x2193)) + "parent");
  m_parentLabel->setFont(font);

  m_topLayout->addWidget(m_childLabel, 1, 0, Qt::AlignTop);
  m_topLayout->addWidget(m_parentLabel, m_topLayout->rowCount() - 1, 0, Qt::AlignBottom);
}

void
TfTransformGraphicsWidget::createContextMenu()
{
  m_createChildAction = new QAction(tr("&Create child"), this);
  connect(m_createChildAction, SIGNAL(triggered()), this, SLOT(createChildWidget()));

  m_deleteAction = new QAction(tr("&Delete this node"), this);
  connect(m_deleteAction, SIGNAL(triggered()), this, SLOT(deleteWidget()));

  m_deleteSubtreeAction = new QAction(tr("Delete &all descendants"), this);
  connect(m_deleteSubtreeAction, SIGNAL(triggered()), this, SLOT(deleteSubtree()));

  m_contextMenu = new QMenu(this);
  m_contextMenu->addAction(m_createChildAction);
  m_contextMenu->addAction(m_deleteAction);
  m_contextMenu->addAction(m_deleteSubtreeAction);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
void
TfTransformGraphicsWidget::createChildWidget()
{
  addChild(m_view->addTfWidget());
  m_view->updateScene();
}

void
TfTransformGraphicsWidget::deleteWidget()
{
  // root can not be deleted
  if (m_parent) {
    m_view->deleteTfWidget(this);
  }
}
/*------------------------------------------------------------------------}}}-*/
