#include <time.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include "ssFrameLib.h"
#include"ioapi.h"
#include"rfans_driver.h"
#include <Eigen/Eigen>
#include <sensor_msgs/Imu.h>
#include<unistd.h>
#include<visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
using namespace std;
std::ofstream file;
namespace rfans_driver
{
size_t packet_size_pcap = 1206;
}
using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher fusion_pointCloud_pub;
sensor_msgs::PointCloud2 msg_pub;
ros::Subscriber single_point_sub;
std::vector<int> num_Pub;
//vector<RFANS_XYZ_S>     point_4={};
void callback_save(const sensor_msgs::PointCloud2ConstPtr& point1, const sensor_msgs::PointCloud2ConstPtr& point2,
                   const sensor_msgs::PointCloud2ConstPtr& point3,const sensor_msgs::PointCloud2ConstPtr& point4)
{
    
    RFANS_XYZ_S*  point_1=new RFANS_XYZ_S[point1->width];
    memcpy(point_1,&point1->data[0],point1->data.size());
    for (int i=0;i<point1->width;i++)
    {

        file<<point_1[i].x<<","<<point_1[i].y<<","<<point_1[i].z<<","<<point_1[i].intent<<","<<point_1[i].timeflag<<","<<point_1[i].laserid<<","<<point_1[i].hangle<<"\n";
    }
    delete []point_1;
    
    RFANS_XYZ_S*  point_2=new RFANS_XYZ_S[point2->width];
    memcpy(point_2,&point2->data[0],point2->data.size());
    for (int i=0;i<point2->width;i++)
    {

        file<<point_2[i].x<<","<<point_2[i].y<<","<<point_2[i].z<<","<<point_2[i].intent<<","<<point_2[i].timeflag<<","<<point_2[i].laserid<<","<<point_2[i].hangle<<"\n";
    }
    delete []point_2;
    
    RFANS_XYZ_S*  point_3=new RFANS_XYZ_S[point3->width];
    memcpy(point_3,&point3->data[0],point3->data.size());
    for (int i=0;i<point3->width;i++)
    {

        file<<point_3[i].x<<","<<point_3[i].y<<","<<point_3[i].z<<","<<point_3[i].intent<<","<<point_3[i].timeflag<<","<<point_3[i].laserid<<","<<point_3[i].hangle<<"\n";
    }
    delete []point_3;
    
    RFANS_XYZ_S*  point_4=new RFANS_XYZ_S[point4->width];
    
    memcpy(point_4,&point4->data[0],point4->data.size());
    for (int i=0;i<point3->width;i++)
    {
        file<<point_4[i].x<<","<<point_4[i].y<<","<<point_4[i].z<<","<<point_4[i].intent<<","<<point_4[i].timeflag<<","<<point_4[i].laserid<<","<<point_4[i].hangle<<"\n";
    }
    delete []point_4;
}



void callback_save(const sensor_msgs::PointCloud2ConstPtr& point1, const sensor_msgs::PointCloud2ConstPtr& point2)
{
    RFANS_XYZ_S*  point_1=new RFANS_XYZ_S[point1->width];
    memcpy(point_1,&point1->data[0],point1->data.size());
    for (int i=0;i<point1->width;i++)
    {

        file<<point_1[i].x<<","<<point_1[i].y<<","<<point_1[i].z<<","<<point_1[i].intent<<","<<point_1[i].timeflag<<","<<point_1[i].laserid<<","<<point_1[i].hangle<<"\n";
    }
    delete []point_1;

    RFANS_XYZ_S*  point_2=new RFANS_XYZ_S[point2->width];
    memcpy(point_2,&point2->data[0],point2->data.size());
    for (int i=0;i<point2->width;i++)
    {

        file<<point_2[i].x<<","<<point_2[i].y<<","<<point_2[i].z<<","<<point_2[i].intent<<","<<point_2[i].timeflag<<","<<point_2[i].laserid<<","<<point_2[i].hangle<<"\n";
    }
    delete []point_2;
}


void callback_save(const sensor_msgs::PointCloud2ConstPtr& point1, const sensor_msgs::PointCloud2ConstPtr& point2,const sensor_msgs::PointCloud2ConstPtr& point3)
{

    RFANS_XYZ_S*  point_1=new RFANS_XYZ_S[point1->width];
    memcpy(point_1,&point1->data[0],point1->data.size());
    for (int i=0;i<point1->width;i++)
    {

        file<<point_1[i].x<<","<<point_1[i].y<<","<<point_1[i].z<<","<<point_1[i].intent<<","<<point_1[i].timeflag<<","<<point_1[i].laserid<<","<<point_1[i].hangle<<"\n";
    }
    delete []point_1;

    RFANS_XYZ_S*  point_2=new RFANS_XYZ_S[point2->width];
    memcpy(point_2,&point2->data[0],point2->data.size());
    for (int i=0;i<point2->width;i++)
    {

        file<<point_2[i].x<<","<<point_2[i].y<<","<<point_2[i].z<<","<<point_2[i].intent<<","<<point_2[i].timeflag<<","<<point_2[i].laserid<<","<<point_2[i].hangle<<"\n";
    }
    delete []point_2;

    RFANS_XYZ_S*  point_3=new RFANS_XYZ_S[point3->width];
    memcpy(point_3,&point3->data[0],point3->data.size());
    for (int i=0;i<point3->width;i++)
    {

        file<<point_3[i].x<<","<<point_3[i].y<<","<<point_3[i].z<<","<<point_3[i].intent<<","<<point_3[i].timeflag<<","<<point_3[i].laserid<<","<<point_3[i].hangle<<"\n";
    }
    delete []point_3;
}


void callback_save(const sensor_msgs::PointCloud2ConstPtr& point1)
{
    RFANS_XYZ_S*  point_1=new RFANS_XYZ_S[point1->width];
    memcpy(point_1,&point1->data[0],point1->data.size());
    for (int i=0;i<point1->width;i++)
    {

        file<<point_1[i].x<<","<<point_1[i].y<<","<<point_1[i].z<<","<<point_1[i].intent<<","<<point_1[i].timeflag<<","<<point_1[i].laserid<<","<<point_1[i].hangle<<"\n";
    }
    delete []point_1;
}







void callback_pub(const sensor_msgs::PointCloud2ConstPtr& point) {
    msg_pub.width = point->width;
    msg_pub.height = 1;
    msg_pub.point_step = sizeof(RFANS_XYZ_S);
    int data_size = msg_pub.width * msg_pub.point_step;
    msg_pub.data.resize(data_size);
    msg_pub.row_step = msg_pub.data.size();
    memcpy(&msg_pub.data[0] , &point->data[0], msg_pub.data.size());
    msg_pub.header.stamp = ros::Time::now();
    fusion_pointCloud_pub.publish(msg_pub);
}






void callback_pub(const sensor_msgs::PointCloud2ConstPtr& point1, const sensor_msgs::PointCloud2ConstPtr& point2,
                  const sensor_msgs::PointCloud2ConstPtr& point3,const sensor_msgs::PointCloud2ConstPtr& point4)
{
    msg_pub.header.frame_id = "world";
    msg_pub.height = 1;
    msg_pub.point_step = sizeof(RFANS_XYZ_S);
    msg_pub.width = point1->width + point2->width + point3->width + point4->width;
    msg_pub.data.resize(msg_pub.width * msg_pub.point_step);
    msg_pub.row_step = msg_pub.data.size();
    memcpy(&msg_pub.data[0], &point1->data[0],point1->data.size());
    memcpy(&msg_pub.data[0]+point1->data.size(), &point2->data[0],point2->data.size());
    memcpy(&msg_pub.data[0]+point1->data.size()+point2->data.size(), &point3->data[0],point3->data.size());
    memcpy(&msg_pub.data[0]+point1->data.size()+point2->data.size()+point3->data.size(), &point4->data[0],point4->data.size());
    fusion_pointCloud_pub.publish(msg_pub);
}


void callback_pub(const sensor_msgs::PointCloud2ConstPtr& point1, const sensor_msgs::PointCloud2ConstPtr& point2,
                  const sensor_msgs::PointCloud2ConstPtr& point3)
{
    msg_pub.header.frame_id = "world";
    msg_pub.height = 1;
    msg_pub.point_step = sizeof(RFANS_XYZ_S);
    msg_pub.width = point1->width + point2->width + point3->width;
    msg_pub.data.resize(msg_pub.width * msg_pub.point_step);
    msg_pub.row_step = msg_pub.data.size();
    memcpy(&msg_pub.data[0], &point1->data[0],point1->data.size());
    memcpy(&msg_pub.data[0]+point1->data.size(), &point2->data[0],point2->data.size());
    memcpy(&msg_pub.data[0]+point1->data.size()+point2->data.size(), &point3->data[0],point3->data.size());
    fusion_pointCloud_pub.publish(msg_pub);
}

void callback_pub(const sensor_msgs::PointCloud2ConstPtr& point1, const sensor_msgs::PointCloud2ConstPtr& point2)
{
    msg_pub.header.frame_id = "world";
    msg_pub.height = 1;
    msg_pub.point_step = sizeof(RFANS_XYZ_S);
    msg_pub.width = point1->width + point2->width;
    msg_pub.data.resize(msg_pub.width * msg_pub.point_step);
    msg_pub.row_step = msg_pub.data.size();
    memcpy(&msg_pub.data[0], &point1->data[0],point1->data.size());
    memcpy(&msg_pub.data[0]+point1->data.size(), &point2->data[0],point2->data.size());
    //ROS_INFO("msg_pub.data.size=%d",(int)msg_pub.width);
    fusion_pointCloud_pub.publish(msg_pub);
}



void InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) {
    static const size_t DataSize = 0;
    initCloud.data.clear();
    initCloud.data.resize( DataSize);
    initCloud.is_bigendian = false ;
    initCloud.fields.resize(11);
    initCloud.is_dense = false;
    int tmpOffset = 0 ;
    for(int i=0; i < initCloud.fields.size() ;i++) {
        switch(i) {
        case 0:
            initCloud.fields[i].name = "x" ;
            initCloud.fields[i].datatype = 7u;
            break;
        case 1:
            initCloud.fields[i].name = "y" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 2:
            initCloud.fields[i].name = "z" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 3:
            initCloud.fields[i].name = "intensity" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 4:
            initCloud.fields[i].name = "laserid" ;
            initCloud.fields[i].datatype = 5u;
            tmpOffset += 4;
            break;
        case 5:
            initCloud.fields[i].name = "timeflag" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 6:
            initCloud.fields[i].name = "hangle" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 7:
            initCloud.fields[i].name = "pulseWidth";
            initCloud.fields[i].datatype = 7u;
            tmpOffset +=4;
            break;
        case 8:
            initCloud.fields[i].name = "range";
            initCloud.fields[i].datatype = 7u;
            tmpOffset +=4;
            break;
        case 9:
            initCloud.fields[i].name = "rol";
            initCloud.fields[i].datatype = 5u;
            tmpOffset +=4;
            break;
        case 10:
            initCloud.fields[i].name = "mirrorid" ;
            initCloud.fields[i].datatype = 2u;
            tmpOffset += 1;
            break;
        }
        initCloud.fields[i].offset = tmpOffset ;
        initCloud.fields[i].count = 1 ;
    }
    initCloud.height = 1;
    initCloud.point_step = sizeof(RFANS_XYZ_S);
    initCloud.row_step = DataSize ;
    initCloud.width = 0 ;
    std::string node_name = ros::this_node::getName();
    std::string frame_id_str = "/world";
    initCloud.header.frame_id = frame_id_str;
}






int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_node");
    ros::NodeHandle nh;
    InitPointcloud2(msg_pub);
    std::vector<string> fuse_sub_string;
    fusion_pointCloud_pub=nh.advertise<sensor_msgs::PointCloud2>("fusion_point",1);
    fuse_sub_string.push_back("/ns1/lidar_points");
    fuse_sub_string.push_back("/ns2/lidar_points");
    fuse_sub_string.push_back("/ns3/lidar_points");
    fuse_sub_string.push_back("/ns4/lidar_points");

    bool BIs_Start[4]={0};
    ros::param::get("/ns1/rfans_driver/Is_Start",BIs_Start[0]);
    ros::param::get("/ns2/rfans_driver/Is_Start",BIs_Start[1]);
    ros::param::get("/ns3/rfans_driver/Is_Start",BIs_Start[2]);
    ros::param::get("/ns4/rfans_driver/Is_Start",BIs_Start[3]);
    
    for (int i=0;i<4;i++)
    {
        if(BIs_Start[i])
        {
            num_Pub.push_back(i);
        }
    }

    bool save;
    nh.param<bool>("save_xyz", save, false);

    string export_path;
    nh.param<string>("OutExport_path", export_path, "/home/bkth/usr/20.txt");

    switch (num_Pub.size())
    {
    case 1:
    {

        if(save)
        {
            file.open(export_path);
            file<<"x"<<","<<"y"<<","<<"z"<<","<<"intent"<<","<<"timeflag"<<","<<"laserid"<<","<<"hangle"<<","<<"mirrorid"<<"\n";
            single_point_sub=nh.subscribe(fuse_sub_string.at(num_Pub.at(0)),1,&callback_pub);
            single_point_sub=nh.subscribe(fuse_sub_string.at(num_Pub.at(0)),1,&callback_save);
            ros::spin();
        }
        else {
            single_point_sub=nh.subscribe(fuse_sub_string.at(num_Pub.at(0)),1,&callback_pub);
            ros::spin();
        }


        break;
    }
    case 2:
    {
        typedef sync_policies::ApproximateTime<PointCloud2,PointCloud2> MySyncPolicy;
        message_filters::Subscriber<sensor_msgs::PointCloud2> point1_sub(nh, fuse_sub_string.at(num_Pub.at(0)), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> point2_sub(nh, fuse_sub_string.at(num_Pub.at(1)), 1);
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point1_sub, point2_sub);
        Synchronizer<MySyncPolicy> sync_save(MySyncPolicy(10), point1_sub, point2_sub);
        if(save)
        {
            file.open(export_path);
            ros::shutdown();
            file<<"x"<<","<<"y"<<","<<"z"<<","<<"intent"<<","<<"timeflag"<<","<<"laserid"<<","<<"hangle"<<","<<"mirrorid"<<"\n";
            ros::AsyncSpinner spinner(2);
            sync.registerCallback(boost::bind(&callback_pub, _1, _2));
            sync_save.registerCallback(boost::bind(&callback_save, _1, _2));
            spinner.start();
            ros::waitForShutdown();
        }
        else {
            sync.registerCallback(boost::bind(&callback_pub, _1, _2));
            ros::spin();
        }
        break;
    }
    case 3:
    {
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2,PointCloud2> MySyncPolicy;
        message_filters::Subscriber<sensor_msgs::PointCloud2> point1_sub(nh, fuse_sub_string.at(num_Pub.at(0)), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> point2_sub(nh, fuse_sub_string.at(num_Pub.at(1)), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> point3_sub(nh, fuse_sub_string.at(num_Pub.at(2)), 1);
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point1_sub, point2_sub,point3_sub);
        Synchronizer<MySyncPolicy> sync_save(MySyncPolicy(10), point1_sub, point2_sub,point3_sub);
        if(save)
        {
            file.open(export_path);
            file<<"x"<<","<<"y"<<","<<"z"<<","<<"intent"<<","<<"timeflag"<<","<<"laserid"<<","<<"hangle"<<","<<"mirrorid"<<"\n";
            ros::AsyncSpinner spinner(2);
            sync.registerCallback(boost::bind(&callback_pub, _1, _2, _3));
            sync_save.registerCallback(boost::bind(&callback_save, _1, _2, _3));
            spinner.start();
            ros::waitForShutdown();
        }
        else {
            sync.registerCallback(boost::bind(&callback_pub, _1, _2, _3));
            ros::spin();
        }
        break;
    }
    case 4:
    {
        typedef sync_policies::ApproximateTime<PointCloud2, PointCloud2,PointCloud2,PointCloud2> MySyncPolicy;
        message_filters::Subscriber<sensor_msgs::PointCloud2> point1_sub(nh, fuse_sub_string.at(num_Pub.at(0)), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> point2_sub(nh, fuse_sub_string.at(num_Pub.at(1)), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> point3_sub(nh, fuse_sub_string.at(num_Pub.at(2)), 1);
        message_filters::Subscriber<sensor_msgs::PointCloud2> point4_sub(nh, fuse_sub_string.at(num_Pub.at(3)), 1);
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point1_sub, point2_sub,point3_sub,point4_sub);
        Synchronizer<MySyncPolicy> sync_save(MySyncPolicy(10), point1_sub, point2_sub,point3_sub,point4_sub);
        if(save)
        {
            file.open(export_path);
            file<<"x"<<","<<"y"<<","<<"z"<<","<<"intent"<<","<<"timeflag"<<","<<"laserid"<<","<<"hangle"<<","<<"mirrorid"<<"\n";
            ros::AsyncSpinner spinner(2);
            sync.registerCallback(boost::bind(&callback_pub, _1, _2, _3,_4));
            sync_save.registerCallback(boost::bind(&callback_save, _1, _2, _3,_4));
            spinner.start();
            ros::waitForShutdown();
        }
        else {
            sync.registerCallback(boost::bind(&callback_pub, _1, _2, _3,_4));
            ros::spin();
        }
        break;
    }
    default:
        ROS_INFO("message of topic is error");
        
    }
    

    return 0;
}



