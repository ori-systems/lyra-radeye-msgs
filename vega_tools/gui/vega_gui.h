#ifndef vega_PANEL_H
#define vega_PANEL_H

// QT
#include <QCheckBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QComboBox>
#include <QSlider>
// ros
//#include "mongodb_store/message_store.h"
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>

// my headers
#include <vega_tools/SignalStrength.h>
#include <radeye/ClearRadiationData.h>
#include <radeye/GenCSV.h>

// standard header
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>

class QLineEdit;
class QPushButton;
class QSlider;

class vegaPanel : public rviz::Panel
{

  Q_OBJECT
public:
  vegaPanel(QWidget *parent = 0);

private:
  ros::NodeHandle n_;

  // pubs and subs
  ros::Publisher take_pic_pub;
  ros::Publisher clear_rad_pub;
  ros::Publisher dump_rad_pub;
  ros::Publisher rtb_pub;
  ros::Publisher front_light_pub;
  ros::Publisher arm_light_pub;

  ros::Subscriber signal_strength_sub;

  ros::ServiceClient save_client;
  ros::ServiceClient delete_client;

  std::string network_name_;
  vega_tools::SignalStrength signal_strength;

  QLineEdit *name_editor_;
  QLineEdit *quality_editor_;
  QLineEdit *signal_editor_;
  QLineEdit *channel_editor_;
  QLabel *name_label_;
  QLabel *quality_label_;
  QLabel *singal_label_;
  QLabel *channel_label_;
  QPushButton *rtb_btn_;
  QPushButton *cancel_rtb_btn_;
  QPushButton *take_picture_btn_;
  QPushButton *gen_csv_btn_;
  QPushButton *clear_pc_btn_;
  QLabel *front_light_label_;
  QPushButton *front_light_btn_;
  QSlider *front_light_slider_;
  QLabel *arm_light_label_;
  QPushButton *arm_light_btn_;
  QSlider *arm_light_slider_;

  void load(const rviz::Config &config);
  void save(rviz::Config config) const;
  void signal_strength_cb(vega_tools::SignalStrength);

  int front_light_val = 0;
  bool front_light_on_off = false;

  int arm_light_val = 0;
  bool arm_light_on_off = false;

public Q_SLOTS:

  void rtb_func();
  void cancel_rtb_func();
  void take_picture_func();
  void save_csv_func();
  void clear_pc_func();
  void front_light_func();
  void front_light_toggle_func();
  void arm_light_func();
  void arm_light_toggle_func();
};

#endif // TELEOP_PANEL_H