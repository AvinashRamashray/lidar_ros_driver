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

    //�༶���ݵĴ���Ĭ�ϣ�
    bool data_level_convert(SHOTS_CALCOUT_S* mtPoint);

    //----------------------0���㷨�����ˣ�-------------------------
    int filterTimeWindow(SHOTS_CALCOUT_S* mtPoint);         // ʱ�䴰�ڹ���
    int filterRangeIntensityWide(SHOTS_CALCOUT_S* mtPoint); // ����Ҷ��������
    int filterAngle(SHOTS_CALCOUT_S* mtPoint);              // �Ƕȹ���
    int filterMultiEchoWide(SHOTS_CALCOUT_S* mtPoint);      // ������ز��������Ӱ��

    //----------------------1���㷨���궨��-------------------------
    //��·�궨
	int reviseRange_singeLaser(SHOTS_CALCOUT_S *mtPoint);
	int reviseWide_singeLaser(SHOTS_CALCOUT_S *mtPoint);
	//int reviseIntensity_singeLaser(SHOTS_CALCOUT_S *mtPoint);	
	int reviseRangeTemprature(SHOTS_CALCOUT_S* mtPoint);  //�����¶ȶԾ����Ӱ��
	int reviseMidFarCalib(SHOTS_CALCOUT_S *mtPoint); //��Զ��������
	//ת��Ϊ12bit�Ҷ�
	int trans12bitIntensity(SHOTS_CALCOUT_S *mtPoint);

	//������ˣ��궨��
	int postRangeFilt(SHOTS_CALCOUT_S *mtPoint);

	//��������
    int reviseRangeIntensity(SHOTS_CALCOUT_S* mtPoint);  //�Ҷȸ���������
    int revisePlusMultiCoef(SHOTS_CALCOUT_S* mtPoint);  //�ӳ�ϵ������
    int reviseRangeConst(SHOTS_CALCOUT_S* mtPoint);  //�ӳ�������

	//��˿���������
	int filterSunNoise_IntensityWide(SHOTS_CALCOUT_S* mtPoint); // ����Ҷ����� ����������
	 //bool filterPoint(size_t no, SHOTS_CALCOUT_S & in_point, const ReviseOptions& reviseOptions);
	bool filterPoint( SHOTS_CALCOUT_S & in_point, const ReviseOptions& reviseOptions);
    //��������
    int reviseWideRange(SHOTS_CALCOUT_S *mtPoint);   //��������������Ӱ��
    int reviseWideLaser(SHOTS_CALCOUT_S *mtPoint);    //������������Ĳ���


    //��λ��������CFans�Ƕȹ���
    int CalbZeroAngle(SHOTS_CALCOUT_S *mtPoint);
    //��ȡ������
    void findCFansMirrorNum_FPGA(SHOTS_CALCOUT_S *currshot);  //FPGA��ȡ
    void  findCFansMirrorNum_cfans(SHOTS_CALCOUT_S * mtPoint) ;    //��λ����ȡ
    //CFans ��Ե�Ƕȹ��ˣ��Ƕ�������
    void findCFansMirrorNum_cfans32(SHOTS_CALCOUT_S *currshot, bool flag);
	void findCFansMirrorNum_wfans64(SHOTS_CALCOUT_S* currshot, bool flag);
    void findCFansMirrorNum_cfans128_v1_0(SHOTS_CALCOUT_S *currshot, bool flag);
    void findCFansMirrorNum_cfans128_v2_0(SHOTS_CALCOUT_S *currshot, bool flag);
    //��ȡ�Ƕ�����
    void findCFansAngleArea( SHOTS_CALCOUT_S *currshot);
    //CFans���ýǶȱ궨
    void reviseAngle(SHOTS_CALCOUT_S *currshot);
	//ƫ����
	void reviseDeltXY(SHOTS_CALCOUT_S* currshot);
	//���Ȼ�����
	void HomogenizationProcess(SHOTS_CALCOUT_S* currshot);
	//ת����������š���ֱ�ǶȺ�ˮƽ�Ƕ�
	void transFilterAnlgeLidarID(SHOTS_CALCOUT_S* currshot);


    //----------------------2���㷨��ת8bit�Ҷȣ�-------------------------
    //12bit�Ҷ�ת��Ϊ8bit�Ҷ�
    unsigned short trans8bitIntensity(SHOTS_CALCOUT_S *mtPoint);

    //��˿��������㷨
    inline bool isNeedFilt() { return need_filt_; }
    inline void setFiltFlag(bool flag = true) { need_filt_ = flag; }
    bool reset();   //�������в���

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

	//�����¶Ȳ���
	bool initTemperPara(const std::vector<float>& revise_map_);
	//���õ�·����궨����
	int set_singleRange_params();
	int setWFansPara_singleRange();     //wfans ����
	int setCFans256Para_singleRange();   //cfans256 ����
	int setRCFansPara_singleRange();     //rfans cfans8 cfans128����
	SINGLE_RANGE_PARA_S getPara_singleRange(uint16_t dataID, uint16_t lidarID, uint16_t planeNum);

	//���õ�·����궨����
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
