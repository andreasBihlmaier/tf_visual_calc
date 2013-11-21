#ifndef _TF_TRANSFORM_WIDGET_H_
#define _TF_TRANSFORM_WIDGET_H_

// system includes

// library includes
#include <QFrame>
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
  :public QFrame
{
  Q_OBJECT
  // properties


  public:
    // enums

    // typedefs

    // const static member variables

    // static utility functions
    static geometry_msgs::TransformStamped toTransformStamped(const tf2::Transform& p_tf, const std::string& p_tfParent, const std::string p_tfName, int p_seq);


    // constructors
    TfTransformWidget(QWidget* p_parent = 0);

    // overwritten methods

    // methods
    void setProxy(QGraphicsProxyWidget* p_proxy);

    // variables


  public slots:
    virtual void broadcastTransform();
    void setTfParent(const QString& p_parent);
    void toggleAbsolute(bool);


  signals:

  protected:
    std::string m_tfName;
    QGridLayout* m_topLayout;

  private:
    // methods
    void createLayout();

    // variables
    tf2::Transform* m_tf;
    tf2::Transform* m_absoluteTf;
    tf2_ros::TransformBroadcaster* m_tfBroadcaster;

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

    QGraphicsProxyWidget* m_proxy;

  private slots:
    void setTfName();

};

#endif // _TF_TRANSFORM_WIDGET_H_
