#ifndef __STAR_SDK_CALC_MULTI_LEVEL_REGISTER
#define __STAR_SDK_CALC_MULTI_LEVEL_REGISTER

#include <sdk-decoder/Star.h>
#include <sdk-calculator/calc/NeighborProcess.h>

#include <string>
#include <sdk-decoder/Configure.h>
#include <vector>
#include <set>

#define MIRRORNUM 4
#define LIDAR16_NUM_ORI 16
#define LIDAR32_NUM_ORI 32
#define LIDAR64_NUM_ORI 64
#define WIDE_COFF1_NUM 2
#define WIDE_COFF2_NUM 5

typedef struct TRANS12BITPARA {
	float wideStart;
	float wideEnd;
	float multiPara[2];
	float constPara[2];	
} TRANS12BITPARA_S;


typedef struct TEMPERPARA{
	float coff2;
	float coff1;
	float coff0;
	float baseTemper;
	float temperMax;
	float temperMin;
}TEMPERPARA_S;

typedef struct SINGLE_RANGE_PARA {
	float  intesityVal;
	float	wideVal;
	float	intensityMin;
	float	wideMin;
	float	wideMax;
	int  currIndex;
	int rangeCoffIndex;
}SINGLE_RANGE_PARA_S;

typedef struct SINGLE_RANGE_LIDAR_PARA {
	float  intesityVal[LIDAR32_NUM_ORI];
	float	wideVal[LIDAR32_NUM_ORI];
	float	intensityMin[LIDAR32_NUM_ORI];
	float	wideMin[LIDAR32_NUM_ORI];
	float	wideMax[LIDAR32_NUM_ORI];
	int  currIndex[LIDAR32_NUM_ORI];
	int rangeCoffIndex[LIDAR32_NUM_ORI];
}SINGLE_RANGE_LIDAR_PARA_S;

typedef struct WIDE_PARA {
	float wideCorrRangeMin;
	float wideCorrRangeMedium;
	float wideCorrRangeMax;
	float wideCoff1[2];
	float wideCoff2[5];
	float intensityCoff1[2];
	float intensityCoff2[5];
}SINGLE_WIDE_PARA_S;


typedef struct SINGLE_WIDE_LIDAR_PARA {
	float wideCorrRangeMin;
	float wideCorrRangeMedium;
	float wideCorrRangeMax;
	float wideCoff1[LIDAR32_NUM_ORI][WIDE_COFF1_NUM];
	float wideCoff2[LIDAR32_NUM_ORI][WIDE_COFF2_NUM];
	float intensityCoff1[LIDAR32_NUM_ORI][WIDE_COFF1_NUM];
	float intensityCoff2[LIDAR32_NUM_ORI][WIDE_COFF2_NUM];

}SINGLE_WIDE_LIDAR_PARA_S;

namespace ss {
namespace calc {

class __star_export MultiLevelRegister{
public:
  

    MultiLevelRegister();
    ~MultiLevelRegister() = default;

    void setup(const Configure& configure);

    int setTemperature(float temperature);
    bool set_intensity_params(std::vector<int>& intent,bool isdefault=true);

    //多级数据的处理（默认）
    bool data_level_convert(SHOTS_CALCOUT_S* mtPoint);

    //----------------------0级算法（过滤）-------------------------
    int filterTimeWindow(SHOTS_CALCOUT_S* mtPoint);         // 时间窗口过滤
    int filterRangeIntensityWide(SHOTS_CALCOUT_S* mtPoint); // 距离灰度脉宽过滤
    int filterAngle(SHOTS_CALCOUT_S* mtPoint);              // 角度过滤
    int filterMultiEchoWide(SHOTS_CALCOUT_S* mtPoint);      // 修正多回波对脉宽的影响

    //----------------------1级算法（标定）-------------------------
    //单路标定
	int reviseRange_singeLaser(SHOTS_CALCOUT_S *mtPoint);
	int reviseWide_singeLaser(SHOTS_CALCOUT_S *mtPoint);
	//int reviseIntensity_singeLaser(SHOTS_CALCOUT_S *mtPoint);	
	int reviseRangeTemprature(SHOTS_CALCOUT_S* mtPoint);  //修正温度对距离的影响
	int reviseMidFarCalib(SHOTS_CALCOUT_S *mtPoint); //中远距离修正
	//转换为12bit灰度
	int trans12bitIntensity(SHOTS_CALCOUT_S *mtPoint);

	//距离过滤（标定后）
	int postRangeFilt(SHOTS_CALCOUT_S *mtPoint);

	//修正距离
    int reviseRangeIntensity(SHOTS_CALCOUT_S* mtPoint);  //灰度改正表修正
    int revisePlusMultiCoef(SHOTS_CALCOUT_S* mtPoint);  //加乘系数修正
    int reviseRangeConst(SHOTS_CALCOUT_S* mtPoint);  //加常数修正

	//拉丝和阳光噪点
	int filterSunNoise_IntensityWide(SHOTS_CALCOUT_S* mtPoint); // 距离灰度脉宽 阳光噪点过滤
	 //bool filterPoint(size_t no, SHOTS_CALCOUT_S & in_point, const ReviseOptions& reviseOptions);
	bool filterPoint( SHOTS_CALCOUT_S & in_point, const ReviseOptions& reviseOptions);
    //修正脉宽
    int reviseWideRange(SHOTS_CALCOUT_S *mtPoint);   //修正距离对脉宽的影响
    int reviseWideLaser(SHOTS_CALCOUT_S *mtPoint);    //修正激光器间的差异


    //零位角修正、CFans角度过滤
    int CalbZeroAngle(SHOTS_CALCOUT_S *mtPoint);
    //获取镜面编号
    void findCFansMirrorNum_FPGA(SHOTS_CALCOUT_S *currshot);  //FPGA获取
    void  findCFansMirrorNum_cfans(SHOTS_CALCOUT_S * mtPoint) ;    //上位机获取
    //CFans 边缘角度过滤（角度修正后）
    void findCFansMirrorNum_cfans32(SHOTS_CALCOUT_S *currshot, bool flag);
	void findCFansMirrorNum_wfans64(SHOTS_CALCOUT_S* currshot, bool flag);
    void findCFansMirrorNum_cfans128_v1_0(SHOTS_CALCOUT_S *currshot, bool flag);
    void findCFansMirrorNum_cfans128_v2_0(SHOTS_CALCOUT_S *currshot, bool flag);
    //获取角度区间
    void findCFansAngleArea( SHOTS_CALCOUT_S *currshot);
    //CFans安置角度标定
    void reviseAngle(SHOTS_CALCOUT_S *currshot);
	//偏心量
	void reviseDeltXY(SHOTS_CALCOUT_S* currshot);
	//均匀化处理
	void HomogenizationProcess(SHOTS_CALCOUT_S* currshot);
	//转换激光器编号、竖直角度和水平角度
	void transFilterAnlgeLidarID(SHOTS_CALCOUT_S* currshot);


    //----------------------2级算法（转8bit灰度）-------------------------
    //12bit灰度转换为8bit灰度
    unsigned short trans8bitIntensity(SHOTS_CALCOUT_S *mtPoint);

    //拉丝处理相关算法
    inline bool isNeedFilt() { return need_filt_; }
    inline void setFiltFlag(bool flag = true) { need_filt_ = flag; }
    bool reset();   //重置所有参数

    ReviseOptions& reviseOptions();
    const ReviseOptions& reviseOptions() const;
    void setReviseOptions(const ReviseOptions& reviseOptions);
	void setDefaultReviseOptions( int32_t data_grade);
	void setDefaultReviseOptions_singleCalib( int32_t data_grade);

	protected:
   

private:
    std::vector<float> revise_map_;
    float temperature_;
    int intensity_duration_[6]{};
    float params_[10]{};

   // std::vector<std::vector<NeighborProcess> >  cloud_procs_vec_;
	std::vector<NeighborProcess>   cloud_procs_vec_;
    bool                                        need_filt_;
    ReviseOptions                               reviseOptions_{};

	//设置温度参数
	bool initTemperPara(const std::vector<float>& revise_map_);
	//设置单路距离标定参数
	int set_singleRange_params();
	int setWFansPara_singleRange();     //wfans 参数
	int setCFans256Para_singleRange();   //cfans256 参数
	int setRCFansPara_singleRange();     //rfans cfans8 cfans128参数
	SINGLE_RANGE_PARA_S getPara_singleRange(uint16_t dataID, uint16_t lidarID, uint16_t planeNum);

	//设置单路脉宽标定参数
	int set_singleWide_params();
	int setCFans256Para_singleWide();
	int setRCFansPara_singleWide();
	SINGLE_WIDE_PARA_S getPara_singleWide(uint16_t dataID, uint16_t lidarID);

	TRANS12BITPARA_S m_trans12bitPara;
	TEMPERPARA_S m_temperPara;

	SINGLE_RANGE_LIDAR_PARA_S m_singleRangePara_wfans[4];
	SINGLE_RANGE_LIDAR_PARA_S m_singleRangePara_cfans256[2];
	SINGLE_RANGE_LIDAR_PARA_S m_singleRangePara;

	SINGLE_WIDE_LIDAR_PARA_S m_singleWidePara;
	SINGLE_WIDE_LIDAR_PARA_S m_singleWidePara_cfans256[2];	
};

}
}

#endif //__STAR_SDK_CALC_MULTI_LEVEL_REGISTER
