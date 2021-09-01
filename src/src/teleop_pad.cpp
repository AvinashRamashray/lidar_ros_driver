#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QComboBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QWidget>
#include "teleop_pad.h"
#include <geometry_msgs/Twist.h>
#include <rfans_driver/RfansCommand.h>
#include <rfans_driver/command.h>
#include <ros/ros.h>

namespace rviz_teleop_commander
{

ros::Publisher subComannd;
ros::Publisher subComannd_ns1;
ros::Publisher subComannd_ns2;
ros::Publisher subComannd_ns3;
ros::Publisher subComannd_ns4;
bool Is_multi;
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
{



  subComannd=nh_.advertise<rfans_driver::command>("contrlComand",1);
  subComannd_ns1=nh_.advertise<rfans_driver::command>("/ns1/contrlComand",1);
  subComannd_ns2=nh_.advertise<rfans_driver::command>("/ns2/contrlComand",1);
  subComannd_ns3=nh_.advertise<rfans_driver::command>("/ns3/contrlComand",1);
  subComannd_ns4=nh_.advertise<rfans_driver::command>("/ns4/contrlComand",1);

  QVBoxLayout* topic_layout = new QVBoxLayout;
  //create a combox of scan speed.
  topic_layout->addWidget( new QLabel( "Scan Speed:" ));
  scan_speed = new QComboBox;
  scan_speed->clear();
  ros::param::get("/model",model);//multi_lidar会覆盖单节点的参数变量

  //ros::param::get("/rfans_driver/rps",rps);
  bool device_num;
  Is_multi=ros::param::get("/device_num",device_num);//multi_lidar会覆盖单节点的参数变量
//  ROS_INFO("Is_multi------=%d",Is_multi);
//  ROS_INFO("Is_multi------=");
  if((model=="R-Fans-32")||(model=="R-Fans-16"))
  {

    scan_speed->addItem("5",0);
    scan_speed->addItem("10",1);
    scan_speed->addItem("20",2);

    if(Is_multi)//启动多台设备
    {
      scan_speed->setCurrentIndex(1);

    }
    else
    {
      if(rps == 5){
        scan_speed->setCurrentIndex(0);
      } else if(rps == 10){
        scan_speed->setCurrentIndex(1);
      } else if(rps == 20){
        scan_speed->setCurrentIndex(2);
      } else {
        scan_speed->setCurrentIndex(1);
      }


    }


  }
  else if((model=="C-Fans-128")||(model=="C-Fans-32"))
  {
    scan_speed->addItem("10",0);
    scan_speed->addItem("20",1);
    scan_speed->addItem("40",2);
    scan_speed->addItem("60",3);
    scan_speed->addItem("80",4);
    if(Is_multi)//启动多台设备
    {
      scan_speed->setCurrentIndex(0);
    }
    else {//启动单台设备
      if(rps == 10){
        scan_speed->setCurrentIndex(0);
      } else if(rps == 20){
        scan_speed->setCurrentIndex(1);
      } else if(rps == 40){
        scan_speed->setCurrentIndex(2);
      } else if(rps == 80){
        scan_speed->setCurrentIndex(4);
      } else {
        scan_speed->setCurrentIndex(1);
      }
    }




  }
  else {
    ROS_INFO("launch model error");
  }

  //  if(flag_rps)
  //  {

  //      ROS_INFO("rps is success=%d",rps);
  //  }
  //  else {
  //      ROS_INFO("rps read error");
  //  }

  topic_layout->addWidget( scan_speed );
  topic_layout->addWidget( new QLabel( "Return Type:" ));
  return_type = new QComboBox;
  return_type->clear();
  return_type->addItem(tr("Strongest return"),0);
  return_type->addItem(tr("Dual return"),1);
  bool double_echo =false;

  ros::param::get("/rfans_driver/use_double_echo",double_echo);//启动多台，无效，命名参数被覆盖

  if(double_echo){
    return_type->setCurrentIndex(1);
  } else {
    return_type->setCurrentIndex(0);
  }
  topic_layout->addWidget( return_type );
  button_ok = new QPushButton;
  button_ok->setText("OK");
  button_ok->setEnabled(true);
  topic_layout->addWidget(button_ok);

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );


  connect(button_ok, SIGNAL(clicked()), this, SLOT(button_clicked()));


}


void TeleopPanel::button_clicked(){
  //    ros::ServiceClient client = nh_.serviceClient<rfans_driver::RfansCommand>("rfans_driver/rfans_control");
  //    rfans_driver::RfansCommand srv;
  //    srv.request.cmd = 1;
  //    srv.request.speed = scan_speed->currentText().toInt();
  //    if(return_type->currentIndex() ==0){
  //        srv.request.use_double_echo = false;
  //    } else {
  //        srv.request.use_double_echo = true;
  //    }
  //    button_ok->setEnabled(false);
  //    if(client.call(srv)){
  //        if(srv.response.status == 1){
  //            button_ok->setEnabled(true);
  //        }
  //    }
  rfans_driver::command command;
  command.cmd = 1;
  command.speed = scan_speed->currentText().toInt();
  if(return_type->currentIndex() ==0){
    command.use_double_echo = false;
  } else {
    command.use_double_echo = true;
  }
  if(Is_multi)//判断是单节点还是双节点
  {
    ROS_INFO("multi_lidar");
    subComannd_ns1.publish(command);
    subComannd_ns2.publish(command);
    subComannd_ns3.publish(command);
    subComannd_ns4.publish(command);
  }
  else
  {
    ROS_INFO("single_lidar");
    subComannd.publish(command);
  }
}

void TeleopPanel::update_Linear_Velocity()
{

  QString temp_string = output_topic_editor_1->text();
  float lin = temp_string.toFloat();
  linear_velocity_ = lin;
}

void TeleopPanel::update_Angular_Velocity()
{
  QString temp_string = output_topic_editor_2->text();
  float ang = temp_string.toFloat() ;
  angular_velocity_ = ang;
}

void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}


void TeleopPanel::setTopic( const QString& new_topic )
{

  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }
    Q_EMIT configChanged();
  }
}


void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}

void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )
// END_TUTORIAL
