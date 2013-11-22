#ifndef _TF_TRANSFORM_WIDGET_H_
#define _TF_TRANSFORM_WIDGET_H_

// system includes

// library includes
#include <QFrame>
#include <QGridLayout>

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <tf2/LinearMath/Transform.h>
# include <tf2_ros/buffer.h>
# include <tf2_ros/transform_broadcaster.h>
# include <tf2_ros/transform_listener.h>
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
  :public QFrame
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions
    static tf2::Transform toTransform(const geometry_msgs::TransformStamped& p_tfStamped);
    static geometry_msgs::TransformStamped toTransformStamped(const tf2::Transform& p_tf, const std::string& p_tfParent, const std::string p_tfName, int p_seq);


    // constructors
    TfTransformWidget(bool p_hasAbsolute = true, QWidget* p_parent = 0);

    // overwritten methods

    // methods
    const std::string& tfName();

    // variables


  public Q_SLOTS:
    virtual void broadcastTransform();
    virtual void toggleAbsolute(bool);
    void setTfParent(const QString& p_parent);


  Q_SIGNALS:


  protected:
    std::string m_tfName;
    QGridLayout* m_topLayout;

  protected Q_SLOTS:
    virtual void setTfName();


  private:
    // methods
    void createLayout();

    // variables
    bool m_hasAbsolute;
    tf2::Transform* m_tf;
    tf2::Transform* m_absoluteTf;
    tf2_ros::Buffer* m_tfBuffer;
    tf2_ros::TransformBroadcaster* m_tfBroadcaster;
    tf2_ros::TransformListener* m_tfListener;

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
    int m_absoluteRepSelectionWidgetRow;
    int m_absoluteRepSelectionWidgetColumn;

  private Q_SLOTS:

};

#endif // _TF_TRANSFORM_WIDGET_H_
