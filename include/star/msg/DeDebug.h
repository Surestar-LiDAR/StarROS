/**
 * @author   lucb
 * @date     2019/12/19
 */

#ifndef __STAR_SDK_MSG_DECODE_DEBUG_H
#define __STAR_SDK_MSG_DECODE_DEBUG_H

#include <star/Debug.h>
#include <star/msg/scd/SCDEchoData.h>
#include <star/msg/scd/SCDSlowData.h>

namespace ss {
namespace msg {

class AngleRangeIntDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);
};

class RiseFallDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const LasShot_S& lasShot, const scd::EchoDataSimplified& echoData);
    void log(const LasShot_S& lasShot, const scd::EchoData& echoData);
};


class PackageConfigDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const scd::ConfigReport& configReport, const PosPPS_S& utcTime);
};

class EnvironmentDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const scd::Environment& environment);
};

class PackageSignDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const scd::PackageSign & package);
    void log(const scd::PackageSign& package, uint64_t totalCount);
};

class SimulationDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const scd::Simulation& simulation);
};

class TimeSynchronDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const scd::Synchron& synchron);
};

class DMIDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const scd::DMI& dmi, const PosPPS_S& utcTime);
};

class CameraDebug : public Debug {
public:
    bool open(const std::string& path, bool enable);

    void log(const scd::Camera& camera);
};

}
}

#endif //__STAR_SDK_MSG_DECODE_DEBUG_H
