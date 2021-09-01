/**
 * @author   lucb
 * @date     2020/2/28
 */

#ifndef __STAR_SDK_MSG_CMD_STAR_PACKET_H
#define __STAR_SDK_MSG_CMD_STAR_PACKET_H

#include <sdk-decoder/Star.h>
#include <sdk-decoder/msg/cmd/Packet.h>
#include <sdk-decoder/utils/factory.h>

 //added by zhubing 2020.4.14
 const uint8_t _except_data_lenth = 0x0F;

namespace ss {
namespace dev {
namespace cmd {
namespace star {

//校验和计算公式
struct __star_export ChecksumCalculator {
public:
    template<typename _Buffer>
    uint16_t calculate(_Buffer& buffer, std::size_t offset, std::size_t length)
    {
        uint32_t sum = 0;
        utils::native_buffer_reader<_Buffer> reader(buffer, false);
        std::size_t pos = reader.tellg();
        reader.seekg(offset);
        while (length > 1u) {
            const uint16_t value = reader.template get_value<uint16_t>();
            sum += value;
            if (sum & 0x80000000)
                sum = (sum & 0xffff) + (sum >> 16);
            length -= 2;
        }
        if (1 == length) {
            sum += reader.template get_value<uint8_t>();
        }
        while (sum >> 16) {
            sum = (sum & 0xffff) + (sum >> 16);
        }
        reader.seekg(pos);
        return sum;
    }
};

//数据包头
class __star_export PacketHeader {
public:
    PacketHeader() = default;

    void setBodyLength(std::size_t length) noexcept;

    std::size_t bodyLength() const noexcept;

    template<typename _Writer>
    ssize_t serialize(_Writer& __writer) noexcept
    {
        if (__writer.size() < this->size()) {
            return -1;
        }
        std::size_t offset = __writer.tellp();
        __writer.put_value(_length);
        __writer.put_value(_version);
        __writer.put_value(_flags);
        __writer.put_value(_enc_padding);
        __writer.put_value(_reservation);
        __writer.put_value(_cmd_set);
        __writer.put_value(_cmd);
        __writer.put_value(_sequence);

        _checksum = _calculator.calculate(__writer.buf(), offset, this->size() - 2);

        __writer.put_value(_checksum);
        return size();
    }

    template<typename _Reader>
    ssize_t deserialize(_Reader& __reader, std::size_t __length) noexcept
    {
        if (__length < size()) {
            return -1;
        }
        const uint16_t checksum = _calculator.calculate(__reader.buf(), __reader.tellg(), this->size() - 2);

        _length = __reader.template get_value<uint16_t>();
        _version = __reader.template get_value<uint8_t>();
        _flags = __reader.template get_value<uint8_t>();
        _enc_padding = __reader.template get_value<uint8_t>();
        _reservation = __reader.template get_value<uint8_t>();
        _cmd_set = __reader.template get_value<uint8_t>();
        _cmd = __reader.template get_value<uint8_t>();
        _sequence = __reader.template get_value<uint16_t>();
        _checksum = __reader.template get_value<uint16_t>();

        if (_checksum != checksum) {
            return -1;
        }
        return size();
    }

    uint16_t length() const noexcept;

    uint8_t version() const noexcept;
    void set_version(uint8_t version);

    uint8_t flags() const noexcept;
    void set_flags(uint8_t flags);

    uint8_t end_padding() const noexcept;
    void set_enc_padding(uint8_t encPadding);

    uint8_t cmd_set() const;
    void set_cmd_set(uint8_t cmdSet);

    uint8_t cmd() const;
    void set_cmd(uint8_t cmd);

    uint16_t sequence() const;
    void set_sequence(uint16_t sequence);

    uint16_t checksum() const;

	/**
     * 区分数据类别
     */
    uint64_t dataId() const noexcept;

    static uint64_t dataId(uint64_t flags, uint64_t cmdset, uint64_t cmd) noexcept;

    /**
     * 序号
     */
    uint64_t identifier() const noexcept;

    static constexpr std::size_t size() noexcept
    {
        return sizeof(_length) + sizeof(_version) + sizeof(_flags) + sizeof(_enc_padding)
            + sizeof(_reservation) + sizeof(_cmd_set) + sizeof(_cmd) + sizeof(_sequence)
            + sizeof(_checksum);
    }

private:
    uint16_t _length;
    uint8_t  _version;
    uint8_t  _flags;
    uint8_t  _enc_padding;
    uint8_t  _reservation;
    uint8_t  _cmd_set;
    uint8_t  _cmd;
    uint16_t _sequence;
    uint16_t _checksum;
    ChecksumCalculator _calculator;
};

template <typename _Tp>
class Data : public msg::cmd::Payload {
public:
    using data_type = _Tp;
    Data() = default;

    explicit Data(const data_type& data) :
        _data(data)
    {
    }

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override
    {
        if (__writer.available() < sizeof(data_type)) {
            return -1;
        }
        __writer.put_value(_data);
        return sizeof(data_type);
    }

    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override
    {
        if (__length < sizeof(data_type)) {
            return -1;
        }
        _data = __reader.get_value<data_type>();
        return sizeof(data_type);
    }

    data_type& data()
    {
        return _data;
    }

    const data_type& data() const
    {
        return _data;
    }

    void set(const data_type& data)
    {
        _data = data;
    }

private:
    data_type _data{ };
};

//added by zhubing 2020.4.26
class __star_export GetBasicReportAck : public msg::cmd::Payload {
public:
	GetBasicReportAck() = default;
	GetBasicReportAck(uint16_t status, uint16_t workModel, uint16_t rangeMin, uint16_t rangeMax, uint16_t echoType, uint16_t mfModel, uint32_t lightPulsation, uint16_t intervalWork, uint16_t fileState, uint16_t scanModel, uint16_t resetSign,
		uint16_t stroragePlace, uint16_t dataType);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t workModel() const;
	void setWorkModel(uint16_t workModel);

	uint16_t rangeMin() const;
	void setRangeMin(uint16_t rangeMin);

	uint16_t rangeMax() const;
	void setRangeMax(uint16_t rangeMax);

	uint16_t echoType() const;
	void setEchoType(uint16_t echoType);

	uint16_t mfModel() const;
	void setMFModel(uint16_t mfModel);

	uint32_t lightPulsation() const;
	void setLightPulsation(uint32_t lightPulsation);

	uint16_t intervalWork() const;
	void setIntervalWork(uint16_t intervalWork);

	uint16_t fileState() const;
	void setFileState(uint16_t fileState);

	uint16_t scanModel() const;
	void setScanModel(uint16_t scanModel);

	uint16_t stroragePlace() const;
	void setStroragePlace(uint16_t stroragePlacey);

	uint16_t dataType() const;
	void setDataType(uint16_t dataType);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _workModel{ 0 };
	uint16_t _rangeMin{ 0 };
	uint16_t _rangeMax{ 0 };
	uint16_t _echoType{ 0 };
	uint16_t _mfModel{ 0 };
	uint32_t _lightPulsation{ 0 };
	uint16_t _intervalWork{ 0 };
	uint16_t _fileState{ 0 };
	uint16_t _scanModel{ 0 };
	uint16_t _resetSign{ 0 };
	uint16_t _stroragePlace{ 0 };
	uint16_t _dataType{ 0 };
};

class __star_export GetLaserReportAck : public msg::cmd::Payload {
public:
	GetLaserReportAck() = default;
	GetLaserReportAck(uint16_t status, uint16_t workState, uint16_t frequency, uint16_t power, uint16_t temperature);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t workState() const;
	void setWorkState(uint16_t workState);

	uint16_t frequency() const;
	void setFrequency(uint16_t frequency);

	uint16_t power() const;
	void setPower(uint16_t power);

	uint16_t temperature() const;
	void setTemperature(uint16_t temperature);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _workState{ 0 };
	uint16_t _frequency{ 0 };
	uint16_t _power{ 0 };
	uint16_t _temperature{ 0 };
};

class __star_export GetScanReportAck : public msg::cmd::Payload {
public:
	GetScanReportAck() = default;
	GetScanReportAck(uint16_t status,  uint16_t workState, uint16_t scanModel, uint16_t speed, uint16_t startRois, uint16_t stopRois, uint16_t speedCount, uint16_t speedState);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t workState() const;
	void setWorkState(uint16_t workState);

	uint16_t scanModel() const;
	void setScanModel(uint16_t scanModel);

	uint16_t speed() const;
	void setSpeed(uint16_t speed);

	uint16_t startRois() const;
	void setStartRois(uint16_t startRois);

	uint16_t stopRois() const;
	void setStopRois(uint16_t stopRois);

	uint16_t speedCount() const;
	void setSpeedCount(uint16_t speedCount);

	uint16_t speedState() const;
	void setSpeedState(uint16_t speedState);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _workState{ 0 };
	uint16_t _scanModel{ 0 };
	uint16_t _speed{ 0 };
	uint16_t _startRois{ 0 };
	uint16_t _stopRois{ 0 };
	uint16_t _speedCount{ 0 };
	uint16_t _speedState{ 0 };
};

class __star_export GetDeviceReportAck : public msg::cmd::Payload {
public:
	GetDeviceReportAck() = default;
	GetDeviceReportAck(uint16_t status, uint16_t rangeMin, uint16_t rangeMax, uint16_t intensityMin, uint16_t intensityMax, uint32_t dataSpeed, uint32_t dataTotal, uint32_t IMPSize, uint16_t dataSpeedFlag, uint16_t dataState, uint16_t freeSpace);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t rangeMin() const;
	void setRangeMin(uint16_t rangeMin);

	uint16_t rangeMax() const;
	void setRangeMax(uint16_t rangeMax);

	uint16_t intensityMin() const;
	void setIntensityMin(uint16_t intensityMin);

	uint16_t intensityMax() const;
	void setIntensityMax(uint16_t intensityMax);

	uint32_t dataSpeed() const;
	void setDataSpeed(uint32_t dataSpeed);

	uint32_t dataTotal() const;
	void setDataTotal(uint32_t dataTotal);

	uint32_t IMPSize() const;
	void setIMPSize(uint32_t IMPSize);

	uint16_t dataSpeedFlag() const;
	void setDataSpeedFlag(uint16_t dataSpeedFlag);

	uint16_t dataState() const;
	void setDataState(uint16_t dataState);

	uint16_t freeSpace() const;
	void setFreeSpace(uint16_t freeSpace);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _rangeMin{ 0 };
	uint16_t _rangeMax{ 0 };
	uint16_t _intensityMin{ 0 };
	uint16_t _intensityMax{ 0 };
	uint32_t _dataSpeed{ 0 };
	uint32_t _dataTotal{ 0 };
	uint32_t _IMPSize{ 0 };
	uint16_t _dataSpeedFlag{ 0 };
	uint16_t _dataState{ 0 };
	uint16_t _freeSpace{ 0 };
};
//end added by zhubing 2020.4.26

//added by zhubing 2020.5.8
class __star_export GetLimitsReportAck : public msg::cmd::Payload {
public:
	GetLimitsReportAck() = default;
	GetLimitsReportAck(uint16_t status, uint16_t deviceType, uint16_t deviceID, uint16_t mpModel, uint16_t rangeMin, uint16_t rangeMax, uint16_t frequencyMin, uint16_t frequencyMax, uint16_t powerMin, uint16_t powerMax, uint16_t scanSpeedMin, uint16_t scanSpeedMax, uint16_t stageSpeedMin, uint16_t stageSpeedMax, uint16_t cameraCount);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t deviceType() const;
	void setDeviceType(uint16_t deviceType);

	uint16_t deviceID() const;
	void setDeviceID(uint16_t deviceID);

	uint16_t mpModel() const;
	void setMPModel(uint16_t mpModel);

	uint16_t rangeMin() const;
	void setRangeMin(uint16_t rangeMin);

	uint16_t rangeMax() const;
	void setRangeMax(uint16_t rangeMax);

	uint16_t frequencyMin() const;
	void setFrequencyMin(uint16_t frequencyMin);

	uint16_t frequencyMax() const;
	void setFrequencyMax(uint16_t frequencyMax);

	uint16_t powerMin() const;
	void setPowerMin(uint16_t powerMin);

	uint16_t powerMax() const;
	void setPowerMax(uint16_t powerMax);

	uint16_t scanSpeedMin() const;
	void setScanSpeedMin(uint16_t scanSpeedMin);

	uint16_t scanSpeedMax() const;
	void setScanSpeedMax(uint16_t scanSpeedMax);

	uint16_t stageSpeedMin() const;
	void setStageSpeedMin(uint16_t stageSpeedMin);

	uint16_t stageSpeedMax() const;
	void setStageSpeedMax(uint16_t stageSpeedMax);

	uint16_t cameraCount() const;
	void setCameraCount(uint16_t cameraCount);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _deviceType{ 0 };
	uint16_t _deviceID{ 0 };
	uint16_t _mpModel{ 0 };
	uint16_t _rangeMin{ 0 };
	uint16_t _rangeMax{ 0 };
	uint16_t _frequencyMin{ 0 };
	uint16_t _frequencyMax{ 0 };
	uint16_t _powerMin{ 0 };
	uint16_t _powerMax{ 0 };
	uint16_t _scanSpeedMin{ 0 };
	uint16_t _scanSpeedMax{ 0 };
	uint16_t _stageSpeedMin{ 0 };
	uint16_t _stageSpeedMax{ 0 };
	uint16_t _cameraCount{ 0 };
};

//end added by zhubing 2020.5.8


//added by zhubing 2020.4.27

class __star_export GetEnvironmentReportAck : public msg::cmd::Payload {
public:
	GetEnvironmentReportAck() = default;
	GetEnvironmentReportAck(uint16_t status, uint16_t GPSType, uint32_t latitude, uint32_t longitude, uint32_t altitude, uint16_t PPSSignal, uint32_t UTCTime, uint32_t inclinometer_X, uint32_t inclinometer_Y, uint32_t temperature_A, uint32_t temperature_B, uint32_t humidity_A, uint32_t humidity_B, uint16_t barometric, uint16_t airspeed);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t GPSType() const;
	void setGPSType(uint16_t GPSType);

	uint16_t PPSSignal() const;
	void setPPSSignal(uint16_t PPSSignal);

	uint32_t UTCTime() const;
	void setUTCTime(uint32_t UTCTime);

	uint32_t inclinometer_X() const;
	void setInclinometer_X(uint32_t inclinometer_X);

	uint32_t inclinometer_Y() const;
	void setInclinometer_Y(uint32_t inclinometer_Y);

	uint32_t temperature_A() const;
	void setTemperature_A(uint32_t temperature_A);

	uint32_t temperature_B() const;
	void setTemperature_B(uint32_t temperature_B);

	uint32_t humidity_A() const;
	void setHumidity_A(uint32_t humidity_A);

	uint32_t humidity_B() const;
	void sethumidity_B(uint32_t humidity_B);

	uint16_t barometric() const;
	void setBarometric(uint16_t barometric);

	uint16_t airspeed() const;
	void setAirspeed(uint16_t airspeed);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _GPSType{ 0 };
	uint16_t _PPSSignal{ 0 };
	uint32_t _latitude{ 0 };
	uint32_t _longitude{ 0 };
	uint32_t _altitude{ 0 };
	uint32_t _UTCTime{ 0 };
	uint32_t _inclinometer_X{ 0 };
	uint32_t _inclinometer_Y{ 0 };
	uint32_t _temperature_A{ 0 };
	uint32_t _temperature_B{ 0 };
	uint32_t _humidity_A{ 0 };
	uint32_t _humidity_B{ 0 };
	uint16_t _barometric{ 0 };
	uint16_t _airspeed{ 0 };
};

//end added by zhubing 2020.4.27


//added by zhubing 2020.4.23
class __star_export CommandBasicParameter : public msg::cmd::Payload {
public:
	CommandBasicParameter() = default;
	CommandBasicParameter(uint16_t workModel, uint16_t rangeMin, uint16_t rangeMax, uint16_t echoType, uint16_t mfModel, uint16_t lightPulsation , uint16_t intervalWork, uint16_t fileState, uint16_t scanModel, uint16_t resetSign,
						uint16_t stroragePlace, uint16_t dataType, uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t miniutes, uint16_t seconds , uint16_t milliseconds );

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t workModel() const;
	void setWorkModels(uint16_t workModel);

	uint16_t rangeMin() const;
	void setRangeMin(uint16_t rangeMin);

	uint16_t rangeMax() const;
	void setRangeMax(uint16_t rangeMax);

	uint16_t echoType() const;
	void setEchoType(uint16_t echoType);

	uint16_t mfModel() const;
	void setMFModel(uint16_t mfModel);

	uint16_t lightPulsation() const;
	void setLightPulsation(uint16_t lightPulsation);

	uint16_t intervalWork() const;
	void setIntervalWork(uint16_t intervalWork);

	uint16_t fileState() const;
	void setFileState(uint16_t fileState);

	uint16_t scanModel() const;
	void setScanModel(uint16_t scanModel);

	uint16_t resetSign() const;
	void setResetSign(uint16_t resetSign);

	uint16_t stroragePlace() const;
	void setStroragePlace(uint16_t stroragePlacey);

	uint16_t dataType() const;
	void setDataType(uint16_t dataType);

	uint16_t year() const;
	void setYear(uint16_t year);

	uint16_t month() const;
	void setMonth(uint16_t month);

	uint16_t day() const;
	void setDay(uint16_t day);

	uint16_t hour() const;
	void setHour(uint16_t hour);

	uint16_t miniutes() const;
	void setMiniutes(uint16_t miniutes);

	uint16_t seconds() const;
	void setSeconds(uint16_t seconds);

	uint16_t milliseconds() const;
	void setMilliseconds(uint16_t milliseconds);

	static std::size_t size();
private:
	uint16_t _workModel{ 0 };
	uint16_t _rangeMin{ 0 };
	uint16_t _rangeMax{ 0 };
	uint16_t _echoType{ 0 };
	uint16_t _mfModel{ 0 };
	uint16_t _lightPulsation{ 0 };
	uint16_t _intervalWork{ 0 };
	uint16_t _fileState{ 0 };
	uint16_t _scanModel{ 0 };
	uint16_t _resetSign{ 0 };
	uint16_t _stroragePlace{ 0 };
	uint16_t _dataType{ 0 };
	uint16_t _year{ 0 };
	uint16_t _month{ 0 };
	uint16_t _day{ 0 };
	uint16_t _hour{ 0 };
	uint16_t _miniutes{ 0 };
	uint16_t _seconds{ 0 };
	uint16_t _milliseconds{ 0 };
};
//end added by zhubing 2020.4.23

//added by zhubing 2020.4.26
class __star_export CommandLaserParameter : public msg::cmd::Payload {
public:
	CommandLaserParameter() = default;
	CommandLaserParameter(uint16_t workModel, uint16_t frequency, uint16_t power, uint16_t temperature);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t workModel() const;
	void setWorkModels(uint16_t workModel);

	uint16_t frequency() const;
	void setFrequency(uint16_t frequency);

	uint16_t power() const;
	void setPower(uint16_t power);

	uint16_t temperature() const;
	void setTemperature(uint16_t temperature);

	static std::size_t size();
private:
	uint16_t _workModel{ 0 };
	uint16_t _frequency{ 0 };
	uint16_t _power{ 0 };
	uint16_t _temperature{ 0 };
};

class __star_export CommandScanParameter : public msg::cmd::Payload {
public:
	CommandScanParameter() = default;
	CommandScanParameter(uint16_t workModel, uint16_t scanModel, uint16_t speed, uint16_t startRois, uint16_t stopRois, uint16_t speedCount, uint16_t speedState);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t workModel() const;
	void setWorkModels(uint16_t workModel);

	uint16_t scanModel() const;
	void setScanModel(uint16_t scanModel);

	uint16_t speed() const;
	void setSpeed(uint16_t speed);

	uint16_t startRois() const;
	void setStartRois(uint16_t startRois);

	uint16_t stopRois() const;
	void setStopRois(uint16_t stopRois);

	uint16_t speedCount() const;
	void setSpeedCount(uint16_t speedCount);

	uint16_t speedState() const;
	void setSpeedState(uint16_t speedState);

	static std::size_t size();
private:
	uint16_t _workModel{ 0 };
	uint16_t _scanModel{ 0 };
	uint16_t _speed{ 0 };
	uint16_t _startRois{ 0 };
	uint16_t _stopRois{ 0 };
	uint16_t _speedCount{ 0 };
	uint16_t _speedState{ 0 };
};


//end added by zhubing 2020.4.26


//定义消息体
class __star_export Command : public Data<uint16_t> {
public:
    Command() = default;
    explicit Command(uint16_t command);

    uint16_t command() const;
    void setCommand(uint16_t command);
};

class __star_export Acknowledge : public Data<uint16_t> {
public:
    Acknowledge() = default;
    explicit Acknowledge(uint16_t status);

    uint16_t status() const;
    void setStatus(uint16_t status);
};

//added by zhubing 2020.4.8
class __star_export GetWorkStateAck : public msg::cmd::Payload {
public:
	GetWorkStateAck() = default;
	GetWorkStateAck(uint16_t status, uint16_t workState);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t workState() const;
	void setWorkState(uint16_t workState);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _workState{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetScanFreqAck : public msg::cmd::Payload {
public:
	GetScanFreqAck() = default;
	GetScanFreqAck(uint16_t status, uint16_t scanFreq);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t scanFreq() const;
	void setScanFreq(uint16_t scanFreq);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _scanFreq{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetScanSpeedAck : public msg::cmd::Payload {
public:
	GetScanSpeedAck() = default;
	GetScanSpeedAck(uint16_t status, uint16_t scanSpeed);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t scanSpeed() const;
	void setScanSpeed(uint16_t scanSpeed);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _scanSpeed{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetRangeMaxAck : public msg::cmd::Payload {
public:
	GetRangeMaxAck() = default;
	GetRangeMaxAck(uint16_t status, uint16_t rangeMax);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t rangeMax() const;
	void setRangeMax(uint16_t rangeMax);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _rangeMax{ 0 };
};

//added by zhubing 2020.4.8
class __star_export GetRangeMinAck : public msg::cmd::Payload {
public:
	GetRangeMinAck() = default;
	GetRangeMinAck(uint16_t status, uint16_t rangeMin);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t rangeMin() const;
	void setRangeMin(uint16_t rangeMin);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _rangeMin{ 0 };
};


class __star_export GetEchoTypeAck : public msg::cmd::Payload {
public:
    GetEchoTypeAck() = default;
    GetEchoTypeAck(uint16_t status, uint16_t echoType);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t echoType() const;
    void setEchoType(uint16_t echoType);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _echoType{ 0 };
};

//added by zhubing 2020.4.20
class __star_export GetPushAPXStateAck : public msg::cmd::Payload {
public:
	GetPushAPXStateAck() = default;
	GetPushAPXStateAck(uint16_t status, uint8_t pushState, uint8_t linkState, uint8_t GPSNumber, uint16_t packetSize);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint8_t pushState() const;
	void setPushState(uint8_t pushState);

	uint8_t linkState() const;
	void setLinkState(uint8_t linkState);

	uint8_t GPSNumber() const;
	void setGPSNumber(uint8_t GPSNumber);

	uint16_t packetSize() const;
	void setPacketSize(uint16_t packetSize);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint8_t  _pushState{ 0 };
	uint8_t  _linkState{ 0 };
	uint8_t  _GPSNumber{ 0 };
	uint16_t _packetSize{ 0 };
};

//added by zhubing 2020.4.20
class __star_export GetPushSDStateAck : public msg::cmd::Payload {
public:
	GetPushSDStateAck() = default;
	GetPushSDStateAck(uint16_t status, uint8_t pushState, uint8_t linkState, uint8_t ISFNumber, uint16_t packetSize);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint8_t pushState() const;
	void setPushState(uint8_t pushState);

	uint8_t linkState() const;
	void setLinkState(uint8_t linkState);

	uint8_t ISFNumber() const;
	void setISFNumber(uint8_t ISFNumber);

	uint16_t packetSize() const;
	void setPacketSize(uint16_t packetSize);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint8_t  _pushState{ 0 };
	uint8_t  _linkState{ 0 };
	uint8_t  _ISFNumber{ 0 };
	uint16_t _packetSize{ 0 };
};

class __star_export GetPushStateAck : public msg::cmd::Payload {
public:
    GetPushStateAck() = default;
    GetPushStateAck(uint16_t status, uint8_t pushState, uint8_t linkState, uint16_t packetSize);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint8_t pushState() const;
    void setPushState(uint8_t pushState);

    uint8_t linkState() const;
    void setLinkState(uint8_t linkState);

    uint16_t packetSize() const;
    void setPacketSize(uint16_t packetSize);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint8_t  _pushState{ 0 };
    uint8_t  _linkState{ 0 };
    uint16_t _packetSize{ 0 };
};

//added by zhubing 2020.3.25
class __star_export GetPushDateAck : public msg::cmd::Payload {
public:
	GetPushDateAck() = default;
	GetPushDateAck(uint16_t status,uint16_t gpsWeek, uint32_t gpsTime, uint64_t latitude, uint64_t longitude, uint64_t altitude, uint32_t northVelocity, uint32_t eastVelocity, uint32_t downVelocity, uint64_t pitch, uint64_t heading, uint64_t roll);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	//added by zhubing 2020.4.20
	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t gpsWeek() const;
	void setgpsWeek(uint16_t gpsWeek);

	uint32_t gpsTime() const;
	void setgpsTime(uint32_t gpsTime);

	uint64_t latitude() const;
	void setLatitude(uint64_t latitude);

	uint64_t longitude() const;
	void setLongitude(uint64_t longitude);

	uint64_t altitude() const;
	void setAltitude(uint64_t altitude);

	uint32_t northVelocity() const;
	void setnorthVelocity(uint32_t northVelocity);

	uint32_t eastVelocity() const;
	void seteastVelocity(uint32_t eastVelocity);

	uint32_t downVelocity() const;
	void setdownVelocity(uint32_t downVelocity);

	uint64_t pitch() const;
	void setPitch(uint64_t pitch);

	uint64_t heading() const;
	void setHeading(uint64_t heading);

	uint64_t roll() const;
	void setRoll(uint64_t roll);

	static std::size_t size();

private:
	uint16_t _status{ 0 };
	uint16_t _gpsWeek{ 0 };
	uint32_t _gpsTime{ 0 };
	uint64_t _latitude{ 0 };
	uint64_t _longitude{ 0 };
	uint64_t _altitude{ 0 };
	uint32_t _northVelocity{ 0 };
	uint32_t _eastVelocity{ 0 };
	uint32_t _downVelocity{ 0 };
	uint64_t _pitch{ 0 };
	uint64_t _heading{ 0 };
	uint64_t _roll{ 0 };
};

//end added by zhubing 2020.3.25

class __star_export GetCameraWorkStateAck : public msg::cmd::Payload {
public:
    GetCameraWorkStateAck() = default;
    GetCameraWorkStateAck(uint16_t status, uint16_t workState);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t workState() const;
    void setWorkState(uint16_t workState);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _workState{ 0 };
};

class __star_export GetCameraWorkModeAck : public msg::cmd::Payload {
public:
    GetCameraWorkModeAck() = default;
    GetCameraWorkModeAck(uint16_t status, uint16_t workMode);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t workMode() const;
    void setWorkMode(uint16_t workMode);

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _workMode{ 0 };
};

class __star_export GetCameraWorkIntervalAck : public msg::cmd::Payload {
public:
    GetCameraWorkIntervalAck() = default;
    GetCameraWorkIntervalAck(uint16_t status, uint16_t workInterval);

    ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
    ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

    uint16_t status() const;
    void setStatus(uint16_t status);

    uint16_t workInterval() const;
    void setWorkInterval(uint16_t workInterval) {_workInterval = workInterval;}

    static std::size_t size();
private:
    uint16_t _status{ 0 };
    uint16_t _workInterval{ 0 };
};

//added by zhubing 2020.4.20
class __star_export GetCameraExposureNumberAck : public msg::cmd::Payload {
public:
	GetCameraExposureNumberAck() = default;
	GetCameraExposureNumberAck(uint16_t status, uint16_t exposureNumber);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t exposureNumber() const;
	void setExposureNumber(uint16_t exposureNumber) { _exposureNumber = exposureNumber; }

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _exposureNumber{ 0 };
};
//added by zhubing 2020.4.27
class __star_export CommandCameraBasicParameter : public msg::cmd::Payload {
public:
	CommandCameraBasicParameter() = default;
	CommandCameraBasicParameter(uint16_t workState, uint16_t workModel, uint16_t workInterval, uint16_t angleOffset, uint16_t messageID, uint32_t flashCount, uint32_t triggerCount, uint32_t startAngle, uint32_t stopAngle);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t workState() const;
	void setWorkState(uint16_t workState);

	uint16_t workModel() const;
	void setWorkModels(uint16_t workModel);

	uint16_t workInterval() const;
	void setWorkInterval(uint16_t workInterval);

	uint16_t angleOffset() const;
	void setAngleOffset(uint16_t angleOffset);

	uint16_t messageID() const;
	void setMessageID(uint16_t messageID);

	uint32_t flashCount() const;
	void setFlashCount(uint32_t flashCount);

	uint32_t triggerCount() const;
	void setTriggerCount(uint32_t triggerCount);

	uint32_t startAngle() const;
	void setStartAngle(uint32_t startAngle);

	uint32_t stopAngle() const;
	void setStopAngle(uint32_t stopAngle);

	static std::size_t size();
private:
	uint16_t _workState{ 0 };
	uint16_t _workModel{ 0 };
	uint16_t _workInterval{ 0 };
	uint16_t _angleOffset{ 0 };
	uint16_t _messageID{ 0 };
	uint32_t _flashCount{ 0 };
	uint32_t _triggerCount{ 0 };
	uint32_t _startAngle{ 0 };
	uint32_t _stopAngle{ 0 };
};

class __star_export GetCameraBasicReportAck : public msg::cmd::Payload {
public:
	GetCameraBasicReportAck() = default;
	GetCameraBasicReportAck(uint16_t status, uint16_t workState, uint16_t workModel, uint16_t workInterval, uint16_t angleOffset, uint16_t messageID, uint32_t flashCount, uint32_t triggerCount, uint32_t startAngle, uint32_t stopAngle);

	ssize_t serialize(utils::buffer_writer& __writer) noexcept override;
	ssize_t deserialize(utils::buffer_reader& __reader, std::size_t __length) noexcept override;

	uint16_t status() const;
	void setStatus(uint16_t status);

	uint16_t workState() const;
	void setWorkState(uint16_t workState);

	uint16_t workModel() const;
	void setWorkModels(uint16_t workModel);

	uint16_t workInterval() const;
	void setWorkInterval(uint16_t workInterval);

	uint16_t angleOffset() const;
	void setAngleOffset(uint16_t angleOffset);

	uint16_t messageID() const;
	void setMessageID(uint16_t messageID);

	uint32_t flashCount() const;
	void setFlashCount(uint32_t flashCount);

	uint32_t triggerCount() const;
	void setTriggerCount(uint32_t triggerCount);

	uint32_t startAngle() const;
	void setStartAngle(uint32_t startAngle);

	uint32_t stopAngle() const;
	void setStopAngle(uint32_t stopAngle);

	static std::size_t size();
private:
	uint16_t _status{ 0 };
	uint16_t _workState{ 0 };
	uint16_t _workModel{ 0 };
	uint16_t _workInterval{ 0 };
	uint16_t _angleOffset{ 0 };
	uint16_t _messageID{ 0 };
	uint32_t _flashCount{ 0 };
	uint32_t _triggerCount{ 0 };
	uint32_t _startAngle{ 0 };
	uint32_t _stopAngle{ 0 };
};

//end added by zhubing 2020.4.27




//定义消息体工厂
class __star_export PacketBodyFactory : public utils::shared_factory<uint64_t, msg::cmd::Payload> {
public:
    using key_type = uint64_t;
    using body_type = std::shared_ptr<msg::cmd::Payload>;
    PacketBodyFactory() noexcept;

    template <typename _Type>
    bool material(uint64_t flags, uint64_t cmdset, uint64_t cmd)
    {
        return utils::shared_factory<uint64_t, msg::cmd::Payload>::material<_Type>(PacketHeader::dataId(flags, cmdset, cmd));
    }

    bool material(uint64_t flags, uint64_t cmdset, uint64_t cmd)
    {
        return utils::shared_factory<uint64_t, msg::cmd::Payload>::material(PacketHeader::dataId(flags, cmdset, cmd));
    }

    std::shared_ptr<msg::cmd::Payload> make(const PacketHeader& header);

    std::size_t max_length(const PacketHeader& header) const;

};

#if 0
class __star_export PacketBodyBuilder {
public:
    using body_type = typename PacketBodyFactory::body_type;
    using key_type  = typename PacketBodyFactory::key_type;

//    using singleton = ss::utils::singleton<PacketBodyBuilder>;

    PacketBodyBuilder() = default;

    template<typename _Head>
    static body_type make(const _Head& head)
    {
        return _factory.make(head);
    }

    template<typename _Head>
    static std::size_t get_max_length(const _Head& head)
    {
        return _factory.max_length(head);
    }

private:
    static PacketBodyFactory _factory;
};
#endif

using PacketTail = msg::cmd::PacketChecksum<uint16_t, ChecksumCalculator, PacketHeader::size() + 1, 0>;

class __star_export Packet : public msg::cmd::Packet<
        msg::cmd::Prefix<uint8_t, static_cast<uint8_t>(0xaau)>,
        PacketHeader,
        PacketBodyFactory,
        PacketTail,
        void>
{
public:
    static constexpr uint8_t CMD = (0x00u << 5u);
    static constexpr uint8_t ACK = (0x01u << 5u);
	static constexpr uint8_t CMD_ERR = 0xFF;

    uint64_t dataId() const noexcept override
    {
        return head().dataId();
    }

    uint64_t identifier() const noexcept override
    {
        return head().identifier();
    }
};


}
}
}
}

#endif //__STAR_SDK_MSG_CMD_STAR_PACKET_H
