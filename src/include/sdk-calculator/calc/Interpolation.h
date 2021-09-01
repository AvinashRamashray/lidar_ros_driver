#ifndef __STAR_SDK_CALC_INTERPOLATION_H
#define __STAR_SDK_CALC_INTERPOLATION_H

#include <sdk-decoder/Star.h>
#include <sdk-decoder/Lidar.h>
#include <sdk-decoder/Shot.h>

#include <string>
#include<string.h>

#include <cmath>

#include <sdk-calculator/calc/math/CTrackFile.h>
#include <sdk-calculator/calc/math/math.h>

namespace ss {
namespace calc {

namespace math {
class CTrackFile;
}

const int POS_REALTIME_SIZE = 4;
typedef struct  {
	math::CTrackFile::SBET_Rec PosRing[POS_REALTIME_SIZE];
	int wrHead;
	int rdTail;
}POS_REALTIME_S;

class __star_export Interpolation {
public:
#if 0
    Interpolation(IMP_DCORING_S* ring);
#endif
    Interpolation();
    ~Interpolation();    

    int setGuiControl(float utc2gps) { cfg_gps_utc = utc2gps; return 0; };
	//	flag 1: fast pps; 0:slw pps
	void updatePpsSign(const PosPPS_S& ppsSign, bool isSlowData);
	int tzero2UTC(unsigned int tzeroStamp, double& utc, int flag = 1);
    double utc2GPS(double utc, double utc_gps);                     

	int setSbetfile(const char* file);
	int gps2POS_m(double gps, PosMsg_S& posMsg);
	//int gps2POS_realTime(double gps, PosMsg_S& posMsg);
//  int tzero2Stage(unsigned int tzeroStamp, TurnMsg_S &stag);   
//	int tzero2Inclt(unsigned int tzeroStamp, IncltMsg_S& inclt);   

	int setUpdatePosFlag(bool flag);
	int updatePos(math::CTrackFile::SBET_Rec PosInfo);
	int findPos_File(double gpst);
	int findPos_realTime(double gpst);
//	int pushSbetBuffer(SBET_Rec* _mt_sbetbuf, unsigned int mt_bufCount);

protected:
    void CheckHeading(double& heading_0, double heading_1);

private:
    PosPPS_S	        _ppsSign;    //ppsSign
    PosPPS_S	        _slwPpsSign; //ppsSign


    std::string	        _sbetpath;

    float		        cfg_gps_utc;		///UTC”ÎGPS ±º‰≤Ó
    float		        cfg_maxUtcGap; ///zks: should be less then shot's t0 cycle (12.8 mSec for 16 bits, 3.2sec for 24 bits)
    int			        s_idxsbet;
    math::CTrackFile*   m_trackFile;

	bool realTimeUpdate;
	math::CTrackFile::SBET_Rec realTimePosInfo[2];
	POS_REALTIME_S _posRealTime;
};

}
}


#endif //__STAR_SDK_CALC_INTERPOLATION_H
