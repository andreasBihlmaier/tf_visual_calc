#include "TfTransformGraphicsWidget.h"

// system includes

// library includes
#include <QLabel>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
TfTransformGraphicsWidget::TfTransformGraphicsWidget(QWidget* p_parent)
  :TfTransformWidget(p_parent)
{
  extendLayout();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected slots: ------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
TfTransformGraphicsWidget::extendLayout()
{
  m_childLabel = new QLabel("^ children"); // use unicode arrow 2191
  m_parentLabel = new QLabel("v parent"); // 2193

  m_topLayout->addWidget(m_childLabel, 1, 0);
  m_topLayout->addWidget(m_parentLabel, m_topLayout->rowCount() - 1, 0);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
