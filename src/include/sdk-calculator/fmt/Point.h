/**
 * @author   lucb
 * @date     2020/1/8
 */

#ifndef __STAR_SDK_FMT_POINTER_H
#define __STAR_SDK_FMT_POINTER_H

#include <sdk-decoder/Star.h>
#include <sdk-decoder/Shot.h>
#include <cstdint>
#include <cstddef>

#pragma pack(2)
namespace ss {
namespace fmt {

struct Color {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

struct Point {
    //Las data
	double   x;
	double   y;
	double   z;
    uint16_t intensity;
    uint8_t  returnNumber:3; //echo number start from 0
    uint8_t  returnCount:3;
    double   gpsTimestamp;
    Color    color;

    //Xyz text data for AngleRange
    double   turnAngle;
    double   angle;
    double   range;
    uint32_t tzero;
    double   pulseWidth;
    uint32_t riseEdge;
    int      mpiaSel;
    double   stageAngle;
    double   bubbleX;
    double   bubbleY;
    int      mirrorNumber;

    bool set(const SHOTS_CALCOUT_S& shot, std::size_t idx);
};

struct Point_LAS {
	//Las data
	long   x;
	long   y;
	long   z;
	uint16_t intensity;
	uint8_t  returnNumber : 3; //echo number start from 0
	uint8_t  returnCount : 3;
	uint8_t  scanDirectionFlag : 1;
	uint8_t  edgeOfFlightLine : 1;
	uint8_t  classification;
	int8_t   scanAngleRank;  //[-90 90] 扫描角度 ，精度2°
	uint8_t  userData;   //lidarID 0-31
	uint16_t sourceId;   //range 点到激光中心的距离
	double   gpsTimestamp;
	Color    color;

	bool set(const SHOTS_CALCOUT_S& shot, std::size_t idx);
};


}
}

#pragma pack()

#endif //__STAR_SDK_FMT_POINTER_H
