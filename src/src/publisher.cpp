/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <ros/ros.h>
#include "rfans_driver.h"
#include <dlfcn.h>
ros::Publisher  sdk_output;
double min_range=0;
double max_range=500;
double min_angle=0;
double max_angle=361;
int ringID;
bool use_laserSelection_;
bool distortion_flag;
float Angle_resolution;
bool Device;
extern Matrix3d R;
extern Matrix3d R;
extern Vector3d TXYZ;
extern double RTX[4];
extern double RTY[4];
extern double RTZ[4];
#define PI 3.14159
void callback(rfans_driver::FilterParamsConfig &config, uint32_t level){
  //ROS_INFO("callback");
  min_range = config.min_range;
  max_range = config.max_range;
  if(Device)
  {
    config.rfans_angleSelection=true;
    min_angle = config.min_angle;
    max_angle = config.max_angle;
    config.cfans_angleSelection =false;
  }
  else
  {
    config.cfans_angleSelection =true;
    min_angle = config.cfans_min_angle;
    max_angle = config.cfans_max_angle;
    config.rfans_angleSelection = false;
  }

  use_laserSelection_ = config.use_laserSelection;
  ringID = config.laserID;
}

void string_date(string& str,vector<float>& num)
{
  char*s_input=(char*) str.c_str();
  const char * split=",";
  char*p=strtok(s_input,split);
  float a;
  while(p!=NULL)
  {
    a=atof(p);
    num.push_back(a);
    p=strtok(NULL,split);//如果第一个参数为空，则函数保存的指针SAVE_ptr再下一次调用作位置；

  };

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rfans_driver");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  rfans_driver::Rfans_Driver* driver = new rfans_driver::Rfans_Driver(node, nh);
  sdk_output = node.advertise<sensor_msgs::PointCloud2>("lidar_points", 10000);
  dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig> server;
  dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig>::CallbackType f;
  f = boost::bind(&callback,_1,_2);
  server.setCallback(f);
  string str;
  vector<float> NUM;
  node.param<string>("RT",str,"0.0,0.0,0.0,0.0,0.0,0.0");
  string_date(str,NUM);
  AngleAxisd Rx(NUM[0]*PI/180,Eigen::Vector3d::UnitX());
  AngleAxisd Ry(NUM[1]*PI/180,Eigen::Vector3d::UnitY());
  AngleAxisd Rz(NUM[2]*PI/180,Eigen::Vector3d::UnitZ());
  R=Rz*Ry*Rx;
  //Vector3d TXYZ1(NUM[3],NUM[4],NUM[5]);
  RTX[0]=R.coeffRef(0,0);
  RTX[1]=R.coeffRef(0,1);
  RTX[2]=R.coeffRef(0,2);
  RTX[3]=NUM[3];


  RTY[0]=R.coeffRef(1,0);
  RTY[1]=R.coeffRef(1,1);
  RTY[2]=R.coeffRef(1,2);
  RTY[3]=NUM[4];

  RTZ[0]=R.coeffRef(2,0);
  RTZ[1]=R.coeffRef(2,1);
  RTZ[2]=R.coeffRef(2,2);
  RTZ[3]=NUM[5];

  //TXYZ=TXYZ1;//3ROW,1ROL
  while (ros::ok()&&driver->spinOnce())
  {
  }


  return 0;
}
