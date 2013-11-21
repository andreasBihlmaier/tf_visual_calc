#include "TfTransformGraphicsWidget.h"

// system includes

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
  m_childLabel = new QLabel(QString(QChar(0x2191)) + "children");
  m_parentLabel = new QLabel(QString(QChar(0x2193)) + "parent");

  m_topLayout->addWidget(m_childLabel, 1, 0, Qt::AlignTop);
  m_topLayout->addWidget(m_parentLabel, m_topLayout->rowCount() - 1, 0, Qt::AlignBottom);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
