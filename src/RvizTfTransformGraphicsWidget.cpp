#include "RvizTfTransformGraphicsWidget.h"

// system includes

// library includes
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QPushButton>
#include <QStyle>
#include <QApplication>
#include <QFileDialog>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
RvizTfTransformGraphicsWidget::RvizTfTransformGraphicsWidget(bool p_hasAbsolute, QWidget* p_parent)
  :TfTransformGraphicsWidget(p_hasAbsolute, p_parent)
{
  extendLayout();
  createMarkerPublisher();
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
  connect(m_markerEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));
  m_markerFileButton = new QPushButton(QApplication::style()->standardIcon(QStyle::SP_FileIcon), tr("&Mesh"));
  connect(m_markerFileButton, SIGNAL(pressed()), this, SLOT(markerDialog()));

  m_markerScaleLabel = new QLabel("Scale");
  m_markerScaleXEdit = new QLineEdit("1.0");
  m_markerScaleXEdit->setFixedWidth(40);
  connect(m_markerScaleXEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));
  m_markerScaleYEdit = new QLineEdit("1.0");
  m_markerScaleYEdit->setFixedWidth(40);
  connect(m_markerScaleYEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));
  m_markerScaleZEdit = new QLineEdit("1.0");
  m_markerScaleZEdit->setFixedWidth(40);
  connect(m_markerScaleZEdit, SIGNAL(editingFinished()), this, SLOT(updateMarker()));

  m_markerLayout = new QHBoxLayout();
  m_markerLayout->addWidget(m_markerLabel);
  m_markerLayout->addWidget(m_markerFileButton);
  m_markerLayout->addWidget(m_markerEdit);
  m_markerLayout->addWidget(m_markerScaleLabel);
  m_markerLayout->addWidget(m_markerScaleXEdit);
  m_markerLayout->addWidget(m_markerScaleYEdit);
  m_markerLayout->addWidget(m_markerScaleZEdit);

  m_topLayout->addLayout(m_markerLayout, m_topLayout->rowCount() + 1, 1);
  updateParentLabel();
}

void
RvizTfTransformGraphicsWidget::createMarkerPublisher()
{
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
void
RvizTfTransformGraphicsWidget::markerDialog()
{
  QString markerFileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open Mesh as Marker"),
                                                        QString(),
                                                        tr("Meshes (*.dae, *.stl)"));
  if (!markerFileName.isEmpty()) {
    m_markerEdit->setText(markerFileName);
    updateMarker();
  }
}

void
RvizTfTransformGraphicsWidget::updateMarker()
{
}
/*------------------------------------------------------------------------}}}-*/
