#ifndef _TF_TRANSFORM_WIDGET_H_
#define _TF_TRANSFORM_WIDGET_H_

// system includes

// library includes
#include <QWidget>
#include <QGridLayout>

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <tf2/LinearMath/Transform.h>
# include <tf2_ros/transform_broadcaster.h>
#endif

// custom includes


// forward declarations
class QHBoxLayout;
class QVBoxLayout;
class QLabel;
class QLineEdit;
class TfTransformRepSelectionWidget;
class QLine;
class QFrame;
class QPushButton;


class TfTransformWidget
  :public QWidget
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions


    // constructors
    TfTransformWidget(QWidget* p_parent = 0);

    // overwritten methods

    // methods

    // variables


  public slots:
    void setTfParent(const QString& p_parent);
    void broadcastTransform();
    void toggleAbsolute(bool);


  signals:

  protected:
    QGridLayout* m_topLayout;

  private:
    // methods
    void createLayout();

    // variables
    tf2::Transform* m_tf;
    tf2::Transform* m_absoluteTf;
    tf2_ros::TransformBroadcaster* m_tfBroadcaster;

    std::string m_tfName;
    std::string m_tfParent;
    unsigned m_broadcastCount;

    QHBoxLayout* m_tfNameLayout;
    QLabel* m_tfNameLabel;
    QLineEdit* m_tfNameEdit;
    QLabel* m_relativeLabel;
    QPushButton* m_absoluteButton;
    QFrame* m_horizontalLineFrame;

    TfTransformRepSelectionWidget* m_relativeRepSelectionWidget;
    TfTransformRepSelectionWidget* m_absoluteRepSelectionWidget;


  private slots:
    void setTfName(const QString& p_name);

};

#endif // _TF_TRANSFORM_WIDGET_H_
