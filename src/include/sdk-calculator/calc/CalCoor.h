#ifndef __STAR_SDK_CALC_CAL_COOR_H
#define __STAR_SDK_CALC_CAL_COOR_H

#include <sdk-decoder/Star.h>
#include <sdk-decoder/Lidar.h>

#include <sdk-calculator/calc/PreFilter.h>
#include <sdk-calculator/calc/MultiLevelRegister.h>
#include <sdk-calculator/calc/CalMultiLidarCoor.h>
#include <sdk-calculator/calc/CalibRange.h>
#include <sdk-calculator/calc/CalibZerosAngle.h>
#include <sdk-calculator/calc/CalOneAxisLidarCoor.h>
#include <sdk-calculator/calc/CalTwoAxisLidarCoor.h>
#include <sdk-calculator/calc/CalGeodeticCoord.h>
#include <sdk-calculator/calc/PostFilter.h>
#include<string.h>

#define M_PI 3.14159265358979323846
//
const int pixelCountW = 1920;
const int pixelCountL = 1080;

typedef struct FRAME_POINT {
	double X;
	double Y;
	double Z;
	unsigned int intensity;
	float angle;
	float range;
	int lidarID;
	double gpsTime;
}FRAME_POINT_S;

//帧投影内参
typedef struct FRAME_PROJECT_PARA {
	float pixelResW;
	float pixelResL;
	float centerX;     //格网中心点的坐标
	float centerY;
	bool firstFrame;   //第一帧的标识
	float minX;
	float maxX;
	float minY;
	float maxY;

}FRAME_PROJECT_PARA_S;


typedef struct {	
	float range[pixelCountW];
	float intent[pixelCountW]; 
	float powerSum[pixelCountW];
}LINE_POINT_S;

typedef struct {	
  LINE_POINT_S linePoints[pixelCountL];
}PROJECT_POWER_S;

typedef struct {
	float gridX[pixelCountW];
	float gridY[pixelCountW];
	float rangeAve[pixelCountW];
	int intentAve[pixelCountW];
}LINE_PROJECT_POINT_S;

typedef struct PROJECT_POINT {
	LINE_PROJECT_POINT_S lineProjPoints[pixelCountL];	
}PROJECT_POINT_S;


//帧投影参数
typedef struct FRAME_TRANSPARA {
	float rotate[3];   //3个旋转角度
	float scale;       //比例尺
	int frameCount;   //帧数
}FRAME_TRANSPARA_S;

namespace ss {
namespace calc {

class Interpolation;

class __star_export CCalCoor
{
public:
    CCalCoor();
    ~CCalCoor();

    void setup(const ss::Configure& configure, Interpolation* interpolation);
    void setDeviceType(int deviceType);
	void setupTemper(float temperature);

    int calcXYZ(SHOTS_CALCOUT_S *currshot) ;           //计算三维坐标

	void outGridData(SHOTS_CALCOUT_S *currshot,FRAME_POINT_S **arrXYZ, float angleRes);

	void pointTransProcess(SHOTS_CALCOUT_S *currshot, PROJECT_POWER_S *project_power);
	void setupTransPara(FRAME_TRANSPARA_S transPara);
	void updateProjectPara(PROJECT_POWER_S *pointPower, PROJECT_POINT *curFrameProject_s, FRAME_TRANSPARA_S transPara, int &curFramNum, int &staticFramNum, FILE *fil);

    MultiLevelRegister& multiLevelRegister();
    CalMultiLidarCoor& calMultiLidarCoor();

private:
    //导航设备坐标
    CalMultiLidarCoor m_calMultiLidarCoor;
    MultiLevelRegister m_levelRegister;
    //测绘设备坐标
    CCalibRange m_calibRange;
    CCalibZerosAngle  m_calibZerosAngle;
    CalOneAxisLidarCoor m_calOneAxisLidarCoor ;
    CalTwoAxisLidarCoor m_calTwoAxisLidarCoor ;
    //大地坐标
    CCalGeodeticCoord m_calGeoeticCoor;
    //过滤
    CPreFilter m_preFilter;
    CPostFilter m_postFilter;

    //硬件参数
    int m_deviceType;

	FRAME_TRANSPARA_S m_transPara_s;
	FRAME_PROJECT_PARA_S m_frame_projPara_s;
	
	
};


}
}



#endif //__STAR_SDK_CALC_CAL_COOR_H
