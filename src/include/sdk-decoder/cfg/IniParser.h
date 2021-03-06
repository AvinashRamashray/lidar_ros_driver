/**
 * @author xiaoma 
 * @date 2020/2/5
 */

#ifndef __STAR_INI_PARSER_H
#define __STAR_INI_PARSER_H

#include <sdk-decoder/cfg/Parser.h>

namespace ss{
namespace cfg {

class __star_export IniReader : public BasicReader {
public:
    IniReader();
    explicit IniReader(const std::string& file);

    void read(Configure& configure) override;

    void readMetadata(Configure& configure, const msg::scd::v3::Metadata& metadata);
protected:
    std::string get_line();
    std::string get_comment_id();
    std::string get_group_name();
    void parse_attribute(Settings& settings);
    void parse_register(Configure& settings);
    void parse_registers(Configure& settings);
    std::string get_name();
    std::string get_value();

    void skipBlank();
    void skipNewLine();
    bool trySkipEscapeNewLine();
};

class __star_export IniWriter : public BasicWriter {
public:
    bool write(const Configure& configure) override;

protected:
    void write(const Settings& settings);
};

class __star_export IniVisitor : public Visitor {
public:
    bool getDeviceVersion(const Configure& configure, DeviceVersion& version) const override;
    void setDeviceVersion(Configure& configure, const DeviceVersion& version) const override;

    bool getDeviceControl(const Configure& configure, DeviceControl& deviceControl) const override;
    void setDeviceControl(Configure& configure, const DeviceControl& deviceControl) const override;

    bool getDeviceConfigure(const Configure& configure, DeviceConfigure& deviceConfigure) const override;
    void setDeviceConfigure(Configure& configure, const DeviceConfigure& deviceConfigure) const override;

    bool getDebugOutput(const Configure& configure, DebugOutput& debugOutput) const override;
    void setDebugOutput(Configure& configure, const DebugOutput& debugOutput) const override;

    bool getRangeIntensityFilter(const Configure& configure, RangeIntensityFilter& rangeIntensityFilter) const override;
    void setRangeIntensityFilter(Configure& configure, const RangeIntensityFilter& rangeIntensityFilter) const override;

    bool getRangeFilter(const Configure& configure, RangeFilter& rangeFilter) const override;
    void setRangeFilter(Configure& configure, const RangeFilter& rangeFilter) const override;

    bool getAngleFilter(const Configure& configur, AngleFilter& angleFiltere) const override;
    void setAngleFilter(Configure& configure, const AngleFilter& angleFilter) const override;

    bool getHeightFilter(const Configure& configure, HeightFilter& heightFilter) const override;
    void setHeightFilter(Configure& configure, const HeightFilter& heightFilter) const override;

    bool getResample(const Configure& configure, Resample& resample) const override;
    void setResample(Configure& configure, const Resample& resample) const override;

    bool getRangeByIntensityFilter(const Configure& configure,
                                   RangeByIntensityFilter& rangeByIntensityFilter) const override;
    void setRangeByIntensityFilter(Configure& configure,
                                   const RangeByIntensityFilter& rangeByIntensityFilter) const override;

    bool getPointFreqRangeFilter(const Configure& configure,
                                 PointFreqAngleRangeFilter& filter) const override;
    void setPointFreqRangeFilter(Configure& configure,
                                 const PointFreqAngleRangeFilter& filter) const  override;

    bool getRangeConstPara(const Configure& configure,
                           RangeConstPara& rangeConstPara) const override;
    void setRangeConstPara(Configure& configure,
                           const RangeConstPara& rangeConstPara) const override;

    bool getLaserRangeConstPara(const Configure& configure,
                                LaserRangeConstPara& laserRangeConstPara) const override;
    void setLaserRangeConstPara(Configure& configure,
                                const LaserRangeConstPara& laserRangeConstPara) const override;

    bool getRangeTemperPara(const Configure& configure, RangeTemperPara& params) const override;
    void setRangeTemperPara(Configure& configure, const RangeTemperPara& params) const override;

    bool getIntensityTable(const Configure& configure, IntensityTable& table) const override;
    void setIntensityTable(Configure& configure, const IntensityTable& table) const override;

    bool getCommonCalibParam(const Configure& configure, CommonCalibParams& params) const override;
    void setCommonCalibParams(Configure& configure, const CommonCalibParams& params) override;

    bool getApCalibParams(const Configure& configure, ApCalibParams& params) const override;
    void setApCalibParams(Configure& configure, const ApCalibParams& params) const override;

    bool getRaCalibParams(const Configure& configure, RaCalibParams& params) const override;
    void setRaCalibParams(Configure& configure, const RaCalibParams& params) const override;

    bool getAkCalibParams(const Configure& configure, AkCalibParams& params) const override;
    void setAkCalibParams(Configure& configure, const AkCalibParams& params) const override;

    bool getUaCalibParams(const Configure& configure, UaCalibParams& params) const override;
    void setUaCalibParams(Configure& configure, const UaCalibParams& uaCalibParams) const override;

    bool getStageCalibParams(const Configure& configure, StageCalibParams& stageCalibParams) const override;
    void setStageCalibParams(Configure& configure, const StageCalibParams& stageCalibParams) const override;

    bool getInterpolation(const Configure& configure, Interpolation& interpolation) const override;
    void setInterpolation(Configure& configure, const Interpolation& interpolation) const override;

    bool getApCmpCoor(const Configure& configure, ApCmpCoor& apCmpCoor) const override;
    void setApCmpCoor(Configure& configure, const ApCmpCoor& apCmpCoor) const override;

    bool getPosCmpCoor(const Configure& configure, PosCmpCoor& posCmpCoor) const override;
    void setPosCmpCoor(Configure& configure, const PosCmpCoor& posCmpCoor) const override;

    bool getGeoProjection(const Configure& configure, GeoProjection& geoProjection) const override;
    void setGeoProjection(Configure& configure, const GeoProjection& geoProjection) const override;

    bool getGpsUAPosition(const Configure& configure, GpsUAPosition& gpsUAPosition) const override;
    void setGpsUAPosition(Configure& configure, const GpsUAPosition& gpsUAPosition) const override;

	bool getOrientUA(const Configure& configure, OrientUA& orientUA) const override;
	void setOrientUA(Configure& configure, const OrientUA& orientUA) const  override;

	bool getAnlignAngleRFans(const Configure& configure, AnlignAngleRFans& anlignAngleRFans) const override;
	bool setAnlignAngleRFans(Configure& configure, AnlignAngleRFans& anlignAngleRFans) const override;
};

}
}

#endif //__STAR_INIPARSER_H
