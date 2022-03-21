#include "vega_gui.h"

vegaPanel::vegaPanel(QWidget *parent) : rviz::Panel(parent)
{

  // pubs & subs
  take_pic_pub = n_.advertise<std_msgs::Bool>("capture_image", 100, true);
  rtb_pub = n_.advertise<std_msgs::Bool>("vega_rtb", 100, true);
  front_light_pub = n_.advertise<std_msgs::UInt8>("vega_front_lights", 100, true);
  arm_light_pub = n_.advertise<std_msgs::UInt8>("vega_arm_lights", 100, true);
  save_client = n_.serviceClient<radeye::GenCSV>("gen_csv");
  delete_client = n_.serviceClient<radeye::ClearRadiationData>("clear_radiation_data");

  signal_strength_sub = n_.subscribe("signal_strength", 10, &vegaPanel::signal_strength_cb, this);

  QHBoxLayout *network_layout = new QHBoxLayout;
  // create element for naming vega
  network_layout->addWidget(new QLabel("Network Name:"));
  name_editor_ = new QLineEdit;
  name_editor_->setReadOnly(true);
  network_layout->addWidget(name_editor_);

  QHBoxLayout *signal_layout = new QHBoxLayout;
  // create element for naming vega
  signal_layout->addWidget(new QLabel("Signal Strength:"));
  signal_editor_ = new QLineEdit;
  signal_editor_->setReadOnly(true);
  signal_layout->addWidget(signal_editor_);

  QHBoxLayout *quality_layout = new QHBoxLayout;
  // create element for naming vega
  quality_layout->addWidget(new QLabel("Signal Quality:"));
  quality_editor_ = new QLineEdit;
  quality_editor_->setReadOnly(true);
  quality_layout->addWidget(quality_editor_);

  QHBoxLayout *channel_layout = new QHBoxLayout;
  // create element for naming vega
  channel_layout->addWidget(new QLabel("Channel:"));
  channel_editor_ = new QLineEdit;
  channel_editor_->setReadOnly(true);
  channel_layout->addWidget(channel_editor_);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(network_layout);
  layout->addLayout(channel_layout);
  layout->addLayout(signal_layout);
  layout->addLayout(quality_layout);

  take_picture_btn_ = new QPushButton(this);
  take_picture_btn_->setText("Take Picture");
  connect(take_picture_btn_, SIGNAL(clicked()), this, SLOT(take_picture_func()));

  gen_csv_btn_ = new QPushButton(this);
  gen_csv_btn_->setText("Save Radiaiton");
  connect(gen_csv_btn_, SIGNAL(clicked()), this, SLOT(save_csv_func()));

  clear_pc_btn_ = new QPushButton(this);
  clear_pc_btn_->setText("Clear Radiation");
  connect(clear_pc_btn_, SIGNAL(clicked()), this, SLOT(clear_pc_func()));

  rtb_btn_ = new QPushButton(this);
  rtb_btn_->setText("Return to base");
  connect(rtb_btn_, SIGNAL(clicked()), this, SLOT(rtb_func()));

  cancel_rtb_btn_ = new QPushButton(this);
  cancel_rtb_btn_->setText("Cancel RTB");
  connect(cancel_rtb_btn_, SIGNAL(clicked()), this, SLOT(cancel_rtb_func()));

  QHBoxLayout *front_light_layout_ = new QHBoxLayout;
  front_light_layout_->addWidget(new QLabel("Front Light Brightness"));
  front_light_slider_ = new QSlider(Qt::Horizontal, this);
  front_light_slider_->setTickPosition(QSlider::TicksBelow);
  front_light_slider_->setTickInterval(1);
  front_light_slider_->setRange(0, 255);
  front_light_slider_->setValue(this->front_light_val);
  connect(front_light_slider_, SIGNAL(valueChanged(int)), this, SLOT(front_light_func()));
  front_light_layout_->addWidget(front_light_slider_);

  front_light_btn_ = new QPushButton(this);
  front_light_btn_->setText("On/Off");
  connect(front_light_btn_, SIGNAL(clicked()), this, SLOT(front_light_toggle_func()));
  front_light_layout_->addWidget(front_light_btn_);

  QHBoxLayout *arm_light_layout_ = new QHBoxLayout;
  arm_light_layout_->addWidget(new QLabel("arm Light Brightness"));
  arm_light_slider_ = new QSlider(Qt::Horizontal, this);
  arm_light_slider_->setTickPosition(QSlider::TicksBelow);
  arm_light_slider_->setTickInterval(1);
  arm_light_slider_->setRange(0, 255);
  arm_light_slider_->setValue(this->arm_light_val);
  connect(arm_light_slider_, SIGNAL(valueChanged(int)), this, SLOT(arm_light_func()));
  arm_light_layout_->addWidget(arm_light_slider_);

  arm_light_btn_ = new QPushButton(this);
  arm_light_btn_->setText("On/Off");
  connect(arm_light_btn_, SIGNAL(clicked()), this, SLOT(arm_light_toggle_func()));
  arm_light_layout_->addWidget(arm_light_btn_);


  // add buttons to the layout
  layout->addWidget(take_picture_btn_);
  layout->addWidget(gen_csv_btn_);
  layout->addWidget(clear_pc_btn_);
  layout->addWidget(rtb_btn_);
  layout->addWidget(cancel_rtb_btn_);
  layout->addLayout(front_light_layout_);
  layout->addLayout(arm_light_layout_);

  setLayout(layout);

  // add_btn_->setEnabled(true);
  // del_btn_->setEnabled(true);
  // gen_sel_btn_->setEnabled(true);
  // gen_path_btn_->setEnabled(true);
  // //db_check_box_->setCheckState(Qt::CheckState(false));
}

void vegaPanel::signal_strength_cb(vega_tools::SignalStrength msg)
{
  this->signal_strength = msg;
  this->name_editor_->setText(QString::fromStdString(msg.name));
  this->quality_editor_->setText(QString::number(msg.quality) + QString::fromStdString("%"));
  this->signal_editor_->setText(QString::number(msg.signal) + QString::fromStdString(" dBm"));
  this->channel_editor_->setText(QString::number(msg.channel));
}

void vegaPanel::rtb_func()
{
  std_msgs::Bool x;
  x.data = true;
  this->rtb_pub.publish(x); // incase anything is driven by this function!!!
}

void vegaPanel::cancel_rtb_func()
{
  std_msgs::Bool x;
  x.data = false;
  this->rtb_pub.publish(x); // incase anything is driven by this function!!!
}

void vegaPanel::take_picture_func()
{
  std_msgs::Bool x;
  x.data = true;
  this->take_pic_pub.publish(x); // incase anything is driven by this function!!!
}

void vegaPanel::save_csv_func()
{
  radeye::GenCSV srv;
  srv.request.data = true;

  if (save_client.call(srv))
  {
    std::cout << srv.response.completed << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service save csv");
  }
}

void vegaPanel::clear_pc_func()
{

  radeye::ClearRadiationData srv;
  srv.request.data = true;

  if (delete_client.call(srv))
  {
    std::cout << srv.response.completed << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to clear radiation point cloud");
  }
}

void vegaPanel::front_light_toggle_func()
{
  this->front_light_on_off = !(this->front_light_on_off);

  std_msgs::UInt8 x;

  if (this->front_light_on_off)
  {
    x.data = this->front_light_val;
  }
  else
  {
    x.data = 0;
  }
  this->front_light_pub.publish(x);
}

void vegaPanel::front_light_func()
{
  this->front_light_val = this->front_light_slider_->value();

  std_msgs::UInt8 x;

  if (this->front_light_on_off)
  {
    x.data = this->front_light_val;
  }
  else
  {
    x.data = 0;
  }
  this->front_light_pub.publish(x);
}


void vegaPanel::arm_light_toggle_func()
{
  this->arm_light_on_off = !(this->arm_light_on_off);

  std_msgs::UInt8 x;

  if (this->arm_light_on_off)
  {
    x.data = this->arm_light_val;
  }
  else
  {
    x.data = 0;
  }
  this->arm_light_pub.publish(x);
}

void vegaPanel::arm_light_func()
{
  this->arm_light_val = this->arm_light_slider_->value();

  std_msgs::UInt8 x;

  if (this->arm_light_on_off)
  {
    x.data = this->arm_light_val;
  }
  else
  {
    x.data = 0;
  }
  this->arm_light_pub.publish(x);
}


void vegaPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}
// Load all configuration data for this panel from the given Config object.
void vegaPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vegaPanel, rviz::Panel)
