/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_FMT_CRS_H
#define __STAR_SDK_FMT_CRS_H

#include <sdk-decoder/Star.h>
#include <sdk-calculator/fmt/Writer.h>
#include <vector>
#include <sdk-decoder/DateTime.h>
#include <sdk-decoder/Lidar.h>

namespace ss {
namespace fmt {

#define  CRS_HEAD_FLAG (0xE7E7E7E7) //!< CRS��ʽ�������ݰ�ͷ��ʶ
#define  CRS_VERSION_FLAG (4)       //!< CRS��ʽ�汾��ʶ

class __star_export CrsBinWriter : public FileWriter{

protected:
	#pragma pack(push, 1)
    struct Timestamp {
        uint16_t wYear;
        uint16_t wMonth;
        uint16_t wDayOfWeek;
        uint16_t wDay;
        uint16_t wHour;
        uint16_t wMinute;
        uint16_t wSecond;
        uint16_t wMilliseconds;
    };
    //CRS ������������ͷ�ṹ
    struct CsrHeader {
        uint32_t   packFlag ;    //!< ���ݰ���ʶ: 0xE7E7E7E7
        uint16_t   versions ;    //!< �汾��
        uint16_t   devNum ;      //!< �豸���
        uint32_t   serialNum ;   //!< ���к�
        uint8_t    devStaFlag ;  //!< �豸״̬��ʶ 0�����������쳣 �� 11 | 11 | 11(������״̬) | 11(ɨ����״̬)
        uint32_t   scanFrq ;     //!< ɨ��Ƶ��   ��λ RPS(ת/��)
        uint32_t   pointFrq ;    //!< ��Ƶ       ��λ Hz(��/��)
        uint32_t   angleRes ;    //!< �Ƕȷֱ��� ��λ ǧ��֮һ��
        uint64_t   pulseNum ;    //!< DMI�������
        uint32_t   pointsCount ; //!< һ�����������ܵ���
        Timestamp  svrSysTime ;     //!< ����ɼ�ʱ�䣬��λʱ�䡣
    };

//CRS �����ݽṹ
    struct CrsPoint {
        float    angle;               //!< ɨ��Ƕ�
        uint16_t range;      //!< ��� ��λmm
        uint16_t intension;  //!< ����ǿ��:
    };


    struct CrsLine {
        CsrHeader             header;
        std::vector<CrsPoint> points;
    };
#pragma pack(pop)

private:
	CrsLine     _crsLine;
	uint64_t    _sumSqrCnt;            //���������ֵ
	uint32_t    _preSqrCnt;            //��һ���������ֵ
	double      _timestamp;
	DateTime    _sysTime;

public:
	CrsBinWriter();
	~CrsBinWriter();

	void writeHeader() override;
	size_t write(const Point& point) override;
	std::string suffix() const override;

	int setupImpHead( cfg::DeviceControl &device);
	//CrsLine getCrsData();
	

	/*void writeHeader() {};

	size_t write(const Point& point) { return 0; };

	std::string suffix()  const
	{
		return "";
	};*/



protected:
	void doWriteHeader();
	void updateHeader(const Point& point);
	void initLinePoints();

};


}
}

#endif //__STAR_SDK_FMT_CRS_H
