#include "TfTransformGraphicsWidget.h"

// system includes
#include <algorithm>

// library includes
#include <QLabel>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
TfTransformGraphicsWidget::TfTransformGraphicsWidget(QWidget* p_parent)
  :TfTransformWidget(p_parent),
   m_parent(NULL)
{
  extendLayout();
}

void
TfTransformGraphicsWidget::addChild(TfTransformGraphicsWidget* p_newChild)
{
  if (std::find(m_children.begin(), m_children.end(), p_newChild) == m_children.end()) {
    m_children.push_back(p_newChild);
    p_newChild->setTfParent(QString::fromStdString(m_tfName));
  }
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
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
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected slots: ------------------------{{{-*/
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
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
