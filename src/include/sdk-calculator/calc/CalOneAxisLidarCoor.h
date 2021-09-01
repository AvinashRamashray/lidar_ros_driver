#ifndef __STAR_SDK_CAL_ONE_AXIS_LIDAR_COOR
#define __STAR_SDK_CAL_ONE_AXIS_LIDAR_COOR

#include <sdk-decoder/Star.h>
#include <sdk-decoder/Configure.h>
#include <sdk-decoder/Shot.h>

namespace ss {
namespace calc {

	struct CalibTransParams {
		double cf[4] = { 0 };
		double	sf[4] = { 0 };
		double	tanV[4] = { 0 };
		double	tanH[4] = { 0 };
		double	sinFai[4] = { 0 };
		double	cosFai[4] = { 0 };
		double	sinKappa[4] = { 0 };
		double	cosKappa[4] = { 0 };
		double q;
		double sinq;
		double cosq;
		double cosVq;
		double sinVq;
	};

class __star_export CalOneAxisLidarCoor
{
public:
    CalOneAxisLidarCoor(void);
    ~CalOneAxisLidarCoor(void);
    bool setup(const ss:: Configure& configure);

    int calXYZ(SHOTS_CALCOUT_S *currshot) ;

protected:
    int cal_apXYZ(SHOTS_CALCOUT_S *currshot);
    //  int cal_tpXYZ(SHOTS_CALCOUT_S *currshot);
    int cal_akXYZ(SHOTS_CALCOUT_S *currshot);
    int cal_raXYZ(SHOTS_CALCOUT_S *currshot);
    int cal_utXYZ(SHOTS_CALCOUT_S *currshot);

private:
    int m_deviceType;
    ss::cfg::CommonCalibParams  m_commonCalibParams;
    ss::cfg::ApCalibParams      m_apCalibParams;
    ss::cfg::RaCalibParams      m_raCalibParams;
    ss::cfg::UaCalibParams      m_uaCalibParams;
    ss::cfg::AkCalibParams      m_akCalibParams;

	CalibTransParams m_calibTransParams;
	
};

}
}

#endif //__STAR_SDK_CAL_ONE_AXIS_LIDAR_COOR
