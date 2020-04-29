/**
 * @author   lucb
 * @date     2019/12/24
 */

#ifndef __STAR_SDK_ISF_V5_MESSAGE_H
#define __STAR_SDK_ISF_V5_MESSAGE_H

#include <star/Star.h>

#include <star/msg/detail/basic_message.h>

#include <star/Shot.h>

#include <cstdint>
#include <cstddef>


namespace ss {
namespace msg {

class ISFReader;

namespace isf {

template <typename _DataType>
class Message : public detail::basic_message<uint8_t, _DataType>{
public:
};

struct Group;
struct Packet;

#if 0
struct LidarPointInfos {
    std::size_t      count;
    LidarPointInfo_S points[2];
};
#endif

struct LasShots {
    std::size_t count;
    LasShot_S   points[2];
};

struct Point {
    uint8_t     grade;
    std::size_t range_count;
    uint16_t    range[2];
    uint16_t    rising_edge;
    std::size_t intensity_count;
    uint16_t    intensity[2];
    uint16_t    pulse_width;

#if 0
    LidarPointInfos toLidarPoints (
        std::size_t groupIdx, 
        std::size_t pointIdx, 
        const Packet& packet, 
        const Group& group
    ) const noexcept;
#endif

    void toLasShots(
        ISFReader& reader,
        LasShots& points,
        std::size_t groupIdx,
        std::size_t pointIdx,
        const Packet& packet,
        const Group& group
    ) const noexcept;
};

struct Group {
    uint8_t     grade;
    uint8_t     mirror_id;
    uint16_t    algorithm;
    uint16_t    azimuth_angle;
	std::size_t point_count;
    Point       points[32];
};

struct Packet {
    uint8_t     grade;
    std::size_t group_count;
    Group       groups[12];
    uint32_t    gps_stamp;
    uint16_t    format;
    uint16_t    device_id;
    uint32_t    stage_angle;
	
    std::size_t getEchoCount() const;
};

class __star_export PointMessage {
public:
    using fast_data = Point;
    virtual fast_data get() const
    {
        fast_data data;
        get(data);
        return data;
    }
	
    virtual void get(fast_data &data) const = 0;
    virtual void set(const fast_data& data) = 0;
};

class __star_export GroupMessage {
public:
    using fast_data = Group;
    virtual fast_data get() const
    {
        fast_data data;
        get(data);
        return data;
    }
	
    virtual void get(fast_data& data) const = 0;
    virtual void set(const fast_data& data) = 0;
};

class __star_export PacketMessage {
public:
    using fast_data = Packet;
    virtual fast_data get() const
    {
        fast_data data;
        get(data);
        return data;
    }
	
    virtual void get(fast_data& data) const = 0;
    virtual void set(const fast_data& data) = 0;
};

}
}
}

#endif //__STAR_SDK_ISF_V5_MESSAGE_H
