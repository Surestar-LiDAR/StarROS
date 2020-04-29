/**
 * @author   lucb
 * @date     2019/12/19
 */

#ifndef __STAR_SDK_SCD_SLOW_DATA_H
#define __STAR_SDK_SCD_SLOW_DATA_H

#include <cstdint>
#include <cstddef>

#include <star/Lidar.h>
#include <star/msg/Message.h>
#include <star/msg/scd/SCDMessage.h>
#include <star/Shot.h>

namespace ss {
namespace msg {
namespace scd {

struct Report {
    uint16_t dataId;
    uint64_t utc;
    uint16_t stamp;
    uint16_t deviceId;
    uint32_t status;
};

struct ConfigReport {
    uint16_t dataId;
    uint64_t utc;
    uint16_t stamp;
    uint32_t content;
    uint16_t DMI_selcount;

    PosPPS_S getUtcTime(lidar::GpsType gpsType, uint32_t stamp, double utcDiff) const;
};

struct Environment {
    uint16_t dataId;
    uint16_t stamp;
    uint32_t height;
    uint32_t speed;
    float temperature[3];
    float humidity[3];
};

struct DMI {
    const static uint32_t MAX_DMI_DATA_SIZE = 20;
    uint16_t dataId;
    uint16_t id;
    uint64_t utc;
    uint32_t t0stamp;
    uint32_t count;
    uint8_t data[MAX_DMI_DATA_SIZE];

    PosPPS_S getUtcTime(lidar::GpsType gpsType, uint32_t stamp, double utcDiff) const;
};

struct Camera {
    uint16_t dataId;
    uint8_t cameraId;
    uint64_t utc;
    uint32_t t0stamp;
    uint32_t flashStamp;
    double turnAngle;

    PosPPS_S getUtcTime(lidar::GpsType gpsType, uint32_t stamp, double utcDiff) const;
    double getTzeroUtcTime();
};

struct IMUPoint {
    uint32_t x;
    uint32_t y;
    uint32_t z;
};

struct IMU {
    uint16_t dataId;
    uint32_t stamp;
    uint32_t sync;

    IMUPoint angular;
    IMUPoint velocity;

    uint16_t gyroStatus;
    uint16_t accelStatus;
};

struct GPS {
    const static uint32_t MAX_GPS_DATA_SIZE = 48;
    uint16_t dataId;
    uint32_t count;
    uint8_t gpsData[MAX_GPS_DATA_SIZE];
};

struct Motor {
    uint16_t dataId;
    uint16_t dataIdCheck;
    uint32_t stamp;
    uint16_t line;
};

struct PackageSign {
    uint32_t dataId; // 0xA7A7A7A7 - zks: being embedded the turret identify within
    uint32_t number;
    uint32_t size;
};

struct Simulation {
    uint32_t dataId; // 0xB7B7B7B7 - zks: being embedded the turret identify within
    uint32_t count;
};

struct Synchron {
    uint32_t dataId;    // 0xE7E7E7E7 -
    uint32_t ppsStamp;
    uint64_t utcStamp;
    uint32_t utcCount;
    uint32_t ppsCount;

    PosPPS_S getUtcTime(lidar::GpsType gpsType, uint32_t stamp, double utcDiff) const;
};

class ReportMessage {
public:
    using fast_data = Report;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class ConfigReportMessage {
public:
    using fast_data = ConfigReport;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class EnvironmentMessage {
public:
    using fast_data = Environment;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class DMIMessage {
public:
    using fast_data = DMI;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class CameraMessage {
public:
    using fast_data = Camera;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class IMUMessage {
public:
    using fast_data = IMU;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class GPSMessage {
public:
    using fast_data = GPS;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class MotorMessage {
public:
    using fast_data = Motor;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class PackageSignMessage {
public:
    using fast_data = PackageSign;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class SimulationMessage {
public:
    using fast_data = Simulation;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;
};

class SynchronMessage {
public:
    using fast_data = Synchron;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& data) = 0;

};

}
}
}

#endif //__STAR_SDK_SCD_SLOW_DATA_H
