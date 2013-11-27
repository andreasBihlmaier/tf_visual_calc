#include "VectorQuaternionGraphicWidget.h"

// system includes

// library includes
#include <QPainter>
#include <QDoubleValidator>

// custom includes
#include "PaintPrimitives.h"


/*---------------------------------- public: -----------------------------{{{-*/
VectorQuaternionGraphicWidget::VectorQuaternionGraphicWidget(QWidget* p_parent)
  :QWidget(p_parent)
{
  setMinimumWidth(350);
  setFixedHeight(135);

  createChildWidgets();
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- public Q_SLOTS: --------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- protected: ----------------------------{{{-*/
void
VectorQuaternionGraphicWidget::paintEvent(QPaintEvent* p_event)
{
  QPainter painter(this);
  int editWidth = 60;
  int editHeight = 25;
  int arrowLineWidth = 3;
  int rotationLineWidth = arrowLineWidth;
  int rotationDiameter = 25;
  int editDistance = 5;

  m_translationLabel->move(width()/4 - m_translationLabel->width()/2, 0);
  m_rotationLabel->move((3*width())/4 - m_rotationLabel->width()/2, 0);

  QPoint translationAxisOrigin(editWidth/2, height() - editHeight/2);
  std::vector<QPoint> translationAxisTips = PaintPrimitives::drawCoordinateAxis(painter, translationAxisOrigin, arrowLineWidth, false);
  QPoint& translationXAxisTip = translationAxisTips[0];
  QPoint& translationYAxisTip = translationAxisTips[1];
  QPoint& translationZAxisTip = translationAxisTips[2];

  m_xEdit->setGeometry(translationXAxisTip.x() + editDistance, translationXAxisTip.y() - editHeight/2, editWidth, editHeight);
  m_yEdit->setGeometry(translationYAxisTip.x() + (editDistance - editWidth/4), translationYAxisTip.y() - (editDistance + editHeight), editWidth, editHeight);
  m_zEdit->setGeometry(translationZAxisTip.x() - editWidth/2, translationZAxisTip.y() - (editDistance + editHeight), editWidth, editHeight);

  QPoint rotationAxisOrigin(translationAxisOrigin.x() + width()/2, height() - editHeight/2);
  std::vector<QPoint> rotationAxisTips = PaintPrimitives::drawCoordinateAxis(painter, rotationAxisOrigin, arrowLineWidth, false);
  QPoint& rotationXAxisTip = rotationAxisTips[0];
  QPoint& rotationYAxisTip = rotationAxisTips[1];
  QPoint& rotationZAxisTip = rotationAxisTips[2];

  m_qxEdit->setGeometry(rotationXAxisTip.x() + editDistance, rotationXAxisTip.y() - editHeight/2, editWidth, editHeight);
  m_qyEdit->setGeometry(rotationYAxisTip.x() + (editDistance - editWidth/4), rotationYAxisTip.y() - (editDistance + editHeight), editWidth, editHeight);
  m_qzEdit->setGeometry(rotationZAxisTip.x() - editWidth/2, rotationZAxisTip.y() - (editDistance + editHeight), editWidth, editHeight);
  int wOffset = 30;
  m_qwEdit->setGeometry(rotationXAxisTip.x() - editWidth/2, rotationXAxisTip.y() - editHeight/2 - wOffset, editWidth, editHeight);
  m_wLabel->move(rotationXAxisTip.x() - editWidth/2 - m_wLabel->width(), rotationXAxisTip.y() - m_wLabel->height()/2 - wOffset);
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
VectorQuaternionGraphicWidget::createChildWidgets()
{
  m_translationLabel = new QLabel("translation", this);
  m_rotationLabel = new QLabel("rotation", this);
  m_wLabel = new QLabel("w", this);

  m_xEdit = new QLineEdit(this);
  m_yEdit = new QLineEdit(this);
  m_zEdit = new QLineEdit(this);

  m_qxEdit = new QLineEdit(this);
  m_qyEdit = new QLineEdit(this);
  m_qzEdit = new QLineEdit(this);
  m_qwEdit = new QLineEdit(this);
}
/*------------------------------------------------------------------------}}}-*/

/*------------------------------- private Q_SLOTS: -------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
