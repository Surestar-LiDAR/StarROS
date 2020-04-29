/**
 * @author   lucb
 * @date     2019/12/19
 */

#ifndef __STAR_SDK_IMP_READER_H
#define __STAR_SDK_IMP_READER_H

#include <star/Star.h>

#include <star/msg/Reader.h>
#include <star/msg/DeDebug.h>

//#include <fstream>

namespace ss {
namespace msg {

class __star_export IMPFilter {
public:
    IMPFilter();
    bool filter(const LasShot_S& point,
        const Configure& configure,
        double intervalInclt);

protected:
    bool checkOutputErrorPoint(const LasShot_S& point,
           const Configure& configure,
           double intervalInclt);
	
private:
    uint32_t _borderT0; //相邻的两个时间点的T0
    uint32_t _checkT0;
    double   _borderScanAngle; //相邻的两个时间点的ScanAngle
    double   _checkScanAngle;
    double   _scanAngleErrorVal;// ScanAngle允许的误差范围
    double   _borderStageAngle; //相邻的两个时间点的StageAngle
    double   _checkStageAngle;
    double   _stageAngleErrorVal;// StageAngle允许的误差范围

    bool        _firstPoint;
    std::size_t _pointsDeleted;
    uint32_t    _lastTimeZero;
    double      _lastScanAngle;
    double      _lastStageAngle;
    bool        _fileFirstPoint;
    std::size_t _filePointCount;
};
	
class __star_export IMPReader : public Reader {
public:
    IMPReader();
    explicit IMPReader(int scd_version);

    void setup(const Configure& configure) override;
    void open_debugs(const std::string& path, const cfg::DebugOutput& parameters) override;
    void update_configure(Configure& configure) override;
    void set_revise(double gpsUtcRevise);

    bool wait_metadata() override;
    void set_metadata(const ss::Configure& configure) override;

protected:
    void setupHeaderHandlers();
    void setupEchoDataHandlers();
    void setupSlowDataHandlers();
    bool verifyPoint(const LasShot_S& point);

    void save_point(const LasShot_S& lasShot) override;

private:
    int                     _scd_version;
    double                  _utc_revise; //this value is from configure file

    double                  _interval_inclt;
    IMPFilter               _filter;
	
    AngleRangeIntDebug      _angleRangeIntDebug;
    RiseFallDebug           _riseFallDebug;
    PackageConfigDebug      _packageConfigDebug;
    EnvironmentDebug        _environmentDebug;
    PackageSignDebug        _packageSignSlowDebug;
    PackageSignDebug        _packageSignFastDebug;
    SimulationDebug         _simulationDebug;
    TimeSynchronDebug       _timeSynchronDebug;
    DMIDebug                _dmiDebug;
    CameraDebug             _cameraDebug;

    std::size_t             _metadata_sequence;
    Configure               _metadata;
};
}
}


#endif //__STAR_SDK_IMP_READER_H
