#include "RvizTfTransformGraphicsWidget.h"

// system includes

// library includes
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
RvizTfTransformGraphicsWidget::RvizTfTransformGraphicsWidget(bool p_hasAbsolute, QWidget* p_parent)
  :TfTransformGraphicsWidget(p_hasAbsolute, p_parent)
{
  extendLayout();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------ protected Q_SLOTS: ------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
RvizTfTransformGraphicsWidget::extendLayout()
{
  m_markerLabel = new QLabel("Marker");
  m_markerEdit = new QLineEdit();

  m_markerLayout = new QHBoxLayout();
  m_markerLayout->addWidget(m_markerLabel);
  m_markerLayout->addWidget(m_markerEdit);
  // TODO add file dialog button and scale (x,y,z) edits

  m_topLayout->addLayout(m_markerLayout, m_topLayout->rowCount() + 1, 1);
  updateParentLabel();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
