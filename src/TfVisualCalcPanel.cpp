#include "TfVisualCalcPanel.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
TfVisualCalcPanel::TfVisualCalcPanel(QWidget* p_parent)
  :rviz::Panel(p_parent)
{
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

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private slots: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TfVisualCalcPanel,rviz::Panel)
