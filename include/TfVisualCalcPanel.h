#ifndef _TF_VISUAL_CALC_PANEL_H_
#define _TF_VISUAL_CALC_PANEL_H_

// system includes

// library includes
#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

// custom includes
#include "TfVisualCalcView.h"


// forward declarations
class QVBoxLayout;



class TfVisualCalcPanel
  : public rviz::Panel
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions


    // constructors
    TfVisualCalcPanel(QWidget* p_parent = 0);

    // overwritten methods
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) /*const*/;

    // methods

    // variables


  public Q_SLOTS:

  protected:

  Q_SIGNALS:


  private:
    // methods

    // variables
    ros::NodeHandle m_nodeHandle;

    QVBoxLayout* m_topLayout;

    TfVisualCalcView* m_calcView;


  private Q_SLOTS:


};

#endif // _TF_VISUAL_CALC_PANEL_H_
