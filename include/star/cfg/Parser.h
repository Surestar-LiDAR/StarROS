/**
/**
 * @author xiaoma
 * @date 2020/2/5
 */

#ifndef __STAR_CFG_PARSER_H
#define __STAR_CFG_PARSER_H

#include <star/Configure.h>

#include <sstream>

namespace ss {
namespace cfg {

/**
 * 配置文件解析语法错误异常对象
 */
class syntax_error : public std::exception {
public:
    syntax_error(int line, int column, const std::string& msg);
    const char* what() const noexcept override;
private:
    std::string _what;
};

#if defined(__GUNC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wundefined-var-template"
#endif

class Reader {
public:
    virtual ~Reader() = default;
    virtual void read(Configure& configure) = 0;
};

class Writer : public Builder<Writer> {
public:
    virtual ~Writer() = default;
    virtual bool write(const Configure& configure) = 0;
};

class BasicReader : public Reader {
public:
    BasicReader();
    explicit BasicReader(const std::string& file);

    void setSourceFile(const std::string& file);
    void setSource(const std::string& source);

    /**
     * 游标，处理出入字符串
     */
    class Cursor {
    public:
        Cursor() = default;
        Cursor(int l, int c, const char* source, std::size_t length, std::size_t offset);

        char            operator*() const;
        Cursor&         operator++();
        const Cursor    operator++(int);
        Cursor&         operator--();
        const Cursor    operator--(int);

        char operator[](std::size_t offset) const;
        void set_source(const char* source, std::size_t length, std::size_t offset = 0);

    protected:
        void move_to_next();
        void move_to_prev();

    public:
        int             line = 1;
        int             column = 1;
    private:
        const char*     _source = nullptr;
        std::size_t     _length = 0;
        std::size_t     _offset = 0;
        int             _lastColumn = 1;
    };

protected:
    Cursor      _cursor;
    std::string _source;
};

class BasicWriter : public Writer {
public:
    virtual bool writeToFile(const std::string& path);
    virtual std::string writeToString();

protected:
    std::stringstream _stream;
};

class Visitor {
public:
    virtual ~Visitor() = default;
    virtual bool updateToStructure(Configure& configure);
    virtual void updateToConfigure(Configure& configure);

    virtual bool getDeviceVersion(const Configure& configure, DeviceVersion& version) const = 0;
    virtual void setDeviceVersion(Configure& configure, const DeviceVersion& version) const = 0;

    virtual bool getDeviceControl(const Configure& configure, DeviceControl& deviceControl) const = 0;
    virtual void setDeviceControl(Configure& configure, const DeviceControl& deviceControl) const = 0;

    virtual bool getDeviceConfigure(const Configure& configure, DeviceConfigure& deviceConfigure) const = 0;
    virtual void setDeviceConfigure(Configure& configure, const DeviceConfigure& deviceConfigure) const = 0;

    virtual bool getDebugOutput(const Configure& configure, DebugOutput& debugOutput) const = 0;
    virtual void setDebugOutput(Configure& configure, const DebugOutput& debugOutput) const = 0;

    virtual bool getRangeIntensityFilter(const Configure& configure, 
        RangeIntensityFilter& rangeIntensityFilter) const = 0;
    virtual void setRangeIntensityFilter(Configure& configure, 
        const RangeIntensityFilter& rangeIntensityFilter) const = 0;

    virtual bool getRangeFilter(const Configure& configure, RangeFilter& rangleFilter) const = 0;
    virtual void setRangeFilter(Configure& configure, const RangeFilter& rangleFilter) const = 0;

//    virtual IntensityFilter intensityFilter(const Configure& configure) const = 0;
//    virtual void setIntensityFilter(Configure& configure, const IntensityFilter& intensityFilter) const = 0;

    virtual bool getAngleFilter(const Configure& configure, AngleFilter& angleFilter) const = 0;
    virtual void setAngleFilter(Configure& configure, const AngleFilter& angleFilter) const = 0;

    virtual bool getHeightFilter(const Configure& configure, HeightFilter& heightFilter) const = 0;
    virtual void setHeightFilter(Configure& configure, const HeightFilter& heightFilter) const = 0;

    virtual bool getResample(const Configure& configure, Resample& resample) const = 0;
    virtual void setResample(Configure& configure, const Resample& resample) const  = 0;

    virtual bool getRangeByIntensityFilter(const Configure& configure, 
        RangeByIntensityFilter& rangeByIntensityFilter) const = 0;
    virtual void setRangeByIntensityFilter(Configure& configure, 
        const RangeByIntensityFilter& rangeByIntensityFilter) const = 0;

    virtual bool getPointFreqRangeFilter(const Configure& configure,
        PointFreqAngleRangeFilter& pointFreqAngleRangeFilter) const = 0;
    virtual void setPointFreqRangeFilter(Configure& configure,
        const PointFreqAngleRangeFilter& pointFreqAngleRangeFilter) const = 0;

    virtual bool getRangeConstPara(const Configure& configure,
        RangeConstPara& rangeConstPara) const  = 0;
    virtual void setRangeConstPara(Configure& configure,
        const RangeConstPara& rangeConstPara) const  = 0;

    virtual bool getLaserRangeConstPara(const Configure& configure, LaserRangeConstPara& laserRangeConstPara) const  = 0;
    virtual void setLaserRangeConstPara(Configure& configure,
            const LaserRangeConstPara& laserRangeConstPara) const  = 0;

    virtual bool getRangeTemperPara(const Configure& configure, RangeTemperPara& rangeTemperPara) const = 0;
    virtual void setRangeTemperPara(Configure& configure, const RangeTemperPara& rangeTemperPara) const = 0;

    virtual bool getIntensityTable(const Configure& configure, IntensityTable& intensityTable) const = 0;
    virtual void setIntensityTable(Configure& configure, const IntensityTable& intensityTable) const = 0;

    virtual bool getCommonCalibParam(const Configure& configure, CommonCalibParams& params) const = 0;
    virtual void setCommonCalibParams(Configure& configure, const CommonCalibParams& params) = 0;

    virtual bool getApCalibParams(const Configure& configure, ApCalibParams& params) const = 0;
    virtual void setApCalibParams(Configure& configure, const ApCalibParams& params) const = 0;

    virtual bool getRaCalibParams(const Configure& configure, RaCalibParams& params) const = 0;
    virtual void setRaCalibParams(Configure& configure, const RaCalibParams& params) const = 0;

    virtual bool getAkCalibParams(const Configure& configure, AkCalibParams& params) const = 0;
    virtual void setAkCalibParams(Configure& configure, const AkCalibParams& params) const = 0;

    virtual bool getUaCalibParams(const Configure& configure, UaCalibParams& uaCalibParams) const = 0;
    virtual void setUaCalibParams(Configure& configure, const UaCalibParams& uaCalibParams) const = 0;

    virtual bool getStageCalibParams(const Configure& configure, StageCalibParams& stageCalibParams) const = 0;
    virtual void setStageCalibParams(Configure& configure, const StageCalibParams& stageCalibParams) const = 0;

    virtual bool getInterpolation(const Configure& configure, Interpolation& interpolation) const = 0;
    virtual void setInterpolation(Configure& configure, const Interpolation& interpolation) const = 0;

    virtual bool getApCmpCoor(const Configure& configure, ApCmpCoor& apCmpCoor) const = 0;
    virtual void setApCmpCoor(Configure& configure, const ApCmpCoor& apCmpCoor) const = 0;

    virtual bool getPosCmpCoor(const Configure& configure, PosCmpCoor& posCmpCoor) const = 0;
    virtual void setPosCmpCoor(Configure& configure, const PosCmpCoor& posCmpCoor) const = 0;

    virtual bool getGeoProjection(const Configure& configure, GeoProjection& geoProjection) const = 0;
    virtual void setGeoProjection(Configure& configure, const GeoProjection& geoProjection) const = 0;

    virtual bool getGpsUAPosition(const Configure& configure, GpsUAPosition& gpsUAPosition) const = 0;
    virtual void setGpsUAPosition(Configure& configure, const GpsUAPosition& gpsUAPosition) const = 0;

    virtual bool getOrientUA(const Configure& configure, OrientUA& orientUA) const = 0;
    virtual void setOrientUA(Configure& configure, const OrientUA& orientUA) const = 0;

	virtual bool getAnlignAngleRFans(const Configure& configure, AnlignAngleRFans& anlignAngleRFans) const =0;
	virtual bool setAnlignAngleRFans(Configure& configure, AnlignAngleRFans& anlignAngleRFans) const =0;

};

}
}

#endif //__STAR_PARSER_H
