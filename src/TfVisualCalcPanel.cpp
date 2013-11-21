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
  rviz::Panel::save(p_config);
  //p_config.mapSetValue("Topic", output_topic_);
}

void
TfVisualCalcPanel::save(rviz::Config p_config) /*const*/
{
  rviz::Panel::load(p_config);
  /*
  QString topic;
  if(p_config.mapGetString("Topic", &topic))
  {
  }
  */
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public slots: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*--------------------------------- protected: ---------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TfVisualCalcPanel,rviz::Panel)
