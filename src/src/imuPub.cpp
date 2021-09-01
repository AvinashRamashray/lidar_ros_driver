#include "ros/ros.h"
#include "std_msgs/String.h"
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <string>
#include <sstream>
#include <iostream>
#include <sdk-decoder/Socket.h>
#include <thread>
#include <stdlib.h>
#include <cstdlib>
#include<rfans_driver/imuData.h>
#include <fstream>
using namespace std;
using namespace ss;
#define IMU_PACKET_NUM 50
#define pi 3.1415926
//byte alignment
#pragma pack(1)
typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day ;
  uint8_t hour;
}TIME_PACKET;
typedef struct
{
  uint16_t flag;
  uint16_t UDPcnt;
  TIME_PACKET UTCtime;
  uint8_t IMUID;
  uint16_t IMUTemperature;
  unsigned char reserve[21];
}IMU_HEADER;//32 byte
typedef struct
{ uint32_t utctime_data;
  int16_t Accel_X;
  int16_t Accel_Y;
  int16_t Accel_Z;
  int16_t Gyro_X;
  int16_t Gyro_Y;
  int16_t Gyro_Z;
  int16_t ROLL;
  int16_t Pitch;
  int16_t Yaw;
}IMU_DATE;//22 bytes

typedef struct
{
  char DataTail[2];
}IMU_TRAILE;//2 bytes
typedef struct
{
  IMU_HEADER header;
  IMU_DATE data[50];
  IMU_TRAILE trail;
}IMU_PACKET;//32+22*50+2=1134byte
#pragma pack()



int ssWeekDay(int yy, int mm, int dd)//year,month,day
{
  int weekDay = 0;

  int i , days = 0 ,s ;
  int mont[13]={0,31,28,31,30,31,30,31,31,30,31,30,31} ;//month has days

  yy = 2000 + yy ;
  //liyp 4年一润，百年不润，400年一润
  if ( ( (yy % 4 == 0) && (yy % 100 !=0) ) || (yy % 400 == 0) ) {//判断闰年，是闰年二月为29天。
    mont[2] = 29 ;
  } else {
    mont[2] = 28 ;
  }
  for( i=0 ; i < mm ; i++ ) { //计算是一年中的第几天
    days += mont[i] ;
  }
  days += dd ;

  s= yy-1+(int)((yy-1)/4)-(int)((yy-1)/100)+(int)((yy-1)/400)+days ;
  weekDay = s % 7 ;
  return weekDay;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imuPub");
  ros::NodeHandle nh;
  TIME_PACKET time;
  rfans_driver::imuData IMUPUB;
  double utcStamp;
  int weekDay_;
  float AccelCoff;
  float GyroCoff;
  float angleCoff;

  ros::Publisher chatter_pub = nh.advertise<rfans_driver::imuData>("ImuTopic", 1000);
  ros::Rate loop_rate(10);

  char buffer[sizeof (IMU_PACKET)];
  memset(buffer,sizeof (IMU_PACKET),1);
  IMU_PACKET* pt;
  IMU_HEADER header_;
  ofstream file;
  file.open("/home/bkth/lc/imuraw_sroll-y.txt");
  Socket udp = Socket::udp();
  if (!udp.bind(InternetEndpoint(2025)) )
  {
    std::cerr << "connect failed" << std::endl;
  }
  bool flag=true;
  while (ros::ok())
  {
    const ssize_t ret = udp.read(buffer, sizeof (buffer));
    if(ret < 0)
    {
      std::cout << "read error:" << std::endl;
      break;
    }
    pt=(IMU_PACKET*)buffer;
    if(flag)
    {

      if(file.is_open())// save utctime
      {
        file<<"year:"<<(int)pt->header.UTCtime.year<<"\n";
        file<<"month:"<<(int)pt->header.UTCtime.month<<"\n";
        file<<"day:"<<(int)pt->header.UTCtime.day<<"\n";
        file<<"hour:"<<(int)pt->header.UTCtime.hour<<"\n";
      }

      /*find imuID,otherwise there is no return value,going to around*/
      if(pt->header.IMUID==0X01)
      {
        AccelCoff=0.061*0.0098;
        GyroCoff=4.37*pi/180*0.001;
        angleCoff=1.0;
        flag=false;
      }else if(pt->header.IMUID==0X02)
      {
        AccelCoff=1.0/2048;
        GyroCoff=125.0/2048;
        angleCoff=45.0/8196;
        flag=false;
      }

    }
    weekDay_=ssWeekDay(pt->header.UTCtime.year,pt->header.UTCtime.month,pt->header.UTCtime.day);
    utcStamp = (double)(weekDay_*86400 + pt->header.UTCtime.hour*3600);
    for (int i=0;i<IMU_PACKET_NUM;i++)
    {
      IMUPUB.time=pt->data[i].utctime_data*0.000001+utcStamp;
      IMUPUB.Accel_X=pt->data[i].Accel_X*AccelCoff;
      IMUPUB.Accel_Y=pt->data[i].Accel_Y*AccelCoff;
      IMUPUB.Accel_Z=pt->data[i].Accel_Z*AccelCoff;
      IMUPUB.Gyro_X=pt->data[i].Gyro_X*GyroCoff;
      IMUPUB.Gyro_Y=pt->data[i].Gyro_Y*GyroCoff;
      IMUPUB.Gyro_Z=pt->data[i].Gyro_Z*GyroCoff;
      IMUPUB.Yaw=pt->data[i].Yaw*angleCoff;
      IMUPUB.Roll=pt->data[i].ROLL*angleCoff;
      IMUPUB.Pitch=pt->data[i].Pitch*angleCoff;

      chatter_pub.publish(IMUPUB);
    }
  }
  file.close();
  return 0;
}
