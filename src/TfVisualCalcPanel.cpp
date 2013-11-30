#include "TfVisualCalcPanel.h"

// system includes
#include <iostream>

// library includes
#include <QVBoxLayout>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
TfVisualCalcPanel::TfVisualCalcPanel(QWidget* p_parent)
  :rviz::Panel(p_parent)
{
  m_topLayout = new QVBoxLayout();

  m_calcView = new TfVisualCalcView();
  m_topLayout->addWidget(m_calcView);

  setLayout(m_topLayout);
}

void
TfVisualCalcPanel::load(const rviz::Config& p_config)
{
  rviz::Panel::load(p_config);
  QString tfVisualCalcViewString;
  if (p_config.mapGetString("TfVisualCalcView", &tfVisualCalcViewString)) {
    m_calcView->fromYAMLString(tfVisualCalcViewString.toStdString());
  }
}

void
TfVisualCalcPanel::save(rviz::Config p_config) const
{
  rviz::Panel::save(p_config);
  p_config.mapSetValue("TfVisualCalcView", QString::fromStdString(m_calcView->toYAMLString()));
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TfVisualCalcPanel,rviz::Panel)
