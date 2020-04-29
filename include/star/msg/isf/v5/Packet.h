/**
 * @author   lucb
 * @date     2019/12/24
 */

#ifndef __STAR_SDK_MSG_ISF_PACKET_H
#define __STAR_SDK_MSG_ISF_PACKET_H

#include <cstdint>

#include <star/msg/isf/ISFPacket.h>

namespace ss {
namespace msg {
namespace isf {
namespace v5 {

static const int GROUP_NUM_ORI = 6;
static const int GROUP_NUM_CALIB = 6;
static const int GROUP_NUM_USER = 10;
static const int GROUP_NUM_USER_SIMPLE = 12;
static const int POINT_NUM_ORI = 32;
static const int POINT_NUM_CALIB = 32;
static const int POINT_NUM_USER = 16;
static const int POINT_NUM_USER_SIMPLE = 32;

// 多级数据打包 start
typedef enum {
    DATA_LEVEL_ORI = 0,     //0级：原始数据
    DATA_LEVEL_CALIB,       //1级：标定数据
    DATA_LEVEL_USER,        //2级：用户级别数据，12bit灰度
    DATA_LEVEL_USER_SIMPLE, //3级：用户级别精简数据
} DATA_LEVEL_E;

typedef enum {
    MIRROR_ID_CFANS_00 = 0x0,   //CFANS ...
    MIRROR_ID_CFANS_01 = 0x1,
    MIRROR_ID_CFANS_02 = 0x2,
    MIRROR_ID_CFANS_03 = 0x3,      //RFANS值默认03镜面标识
} MIRROR_ID_E;

#pragma pack(push, 1)

// 0级数据封装：
typedef struct {
    uint16_t range;
    uint16_t rising_edge;
    uint8_t  intents[3]; //{Intensity[23:12]，Pulse Widt[11:0]}
} POINT_ORI_S;

typedef struct {
    uint16_t    mirror_id: 2;
    uint16_t    data_grade : 2;
    uint16_t    calculate_encode : 12;
    uint16_t    azimuth_angle;
    POINT_ORI_S points[POINT_NUM_ORI];
} GROUP_ORI_S;

typedef struct {
    GROUP_ORI_S groups[GROUP_NUM_ORI];
    uint8_t     reserved[32];
    uint32_t    gps_timestamp;
    uint8_t     packet_fromat;
    uint8_t     device_id;
} PACKET_ORI_S;

//1级数据封装同0级数据
//2级数据封装：12bit intensity
typedef struct {
    uint16_t range1;
    uint16_t range2;
    uint8_t  intents[3]; //{intensity1[23:12]，intensity2[11:0]}
} POINT_USER_S;           // contains 2 point

typedef struct {
    uint16_t        mirror_id : 2;
    uint16_t        data_grade : 2;
    uint16_t        calculate_encode : 12;
    uint16_t        azimuth_angle;
    POINT_USER_S    points[POINT_NUM_USER];
} GROUP_USER_S;

typedef struct {
    GROUP_USER_S    groups[GROUP_NUM_USER];
    uint8_t         reserved[40];
    uint32_t        gps_timestamp;
    uint8_t         packet_fromat;
    uint8_t         device_id;
} PACKET_USER_S;

//3级数据封装：
typedef struct {
    uint16_t range;
    uint8_t  intensity;
} POINT_USER_SIMPLE_S;

typedef struct {
    uint16_t                mirror_id : 2;
    uint16_t                data_grade : 2;
    uint16_t                calculate_encode : 12;
    uint16_t                azimuth_angle;
    POINT_USER_SIMPLE_S     points[POINT_NUM_USER_SIMPLE];
} GROUP_USER_SIMPLE_S;

typedef struct {
    GROUP_USER_SIMPLE_S groups[GROUP_NUM_USER_SIMPLE];
    uint32_t            gps_timestamp;
    uint8_t             packet_fromat;
    uint8_t             device_id;
} PACKET_USER_SIMPLE_S;

#pragma pack(pop)

class PointOrigin : public isf::Message<POINT_ORI_S>,
    public isf::PointMessage {
public:
    inline uint64_t dataId() const override
    {
        return 0u;
    }

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

class GroupOrigin : public isf::Message<GROUP_ORI_S>,
    public isf::GroupMessage {
public:
    inline uint64_t dataId() const override
    {
        return 0u;
    }

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

class PacketOrigin : public isf::Message<PACKET_ORI_S>,
    public isf::PacketMessage {
public:
    inline uint64_t dataId() const override
    {
        return 0u;
    }
    
    ssize_t decode(ss::utils::buffer_reader& __reader) override;

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

class PointUser : public isf::Message<POINT_USER_S>,
    public isf::PointMessage {
public:
    inline uint64_t dataId() const override
    {
        return 2u;
    }

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

class GroupUser : public isf::Message<GROUP_USER_S>,
    public isf::GroupMessage {
public:
    inline uint64_t dataId() const override
    {
        return 2u;
    }

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

class PacketUser : public isf::Message<PACKET_USER_S>,
    public isf::PacketMessage {
public:
    inline uint64_t dataId() const override
    {
        return 2u;
    }

    ssize_t decode(ss::utils::buffer_reader& __reader) override;

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};


class PointUserSimple : public isf::Message<POINT_USER_SIMPLE_S>,
    public isf::PointMessage {
public:
    inline uint64_t dataId() const override
    {
        return 3u;
    }

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

class GroupUserSimple : public isf::Message<GROUP_USER_SIMPLE_S>,
    public isf::GroupMessage {
public:
    inline uint64_t dataId() const override
    {
        return 3u;
    }

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

class PacketUserSimple : public isf::Message<PACKET_USER_SIMPLE_S>,
    public isf::PacketMessage {
public:
    inline uint64_t dataId() const override
    {
        return 3u;
    }

    ssize_t decode(ss::utils::buffer_reader& __reader) override;

    void get(fast_data& data) const override;
    void set(const fast_data& data) override;
};

}
}
}
}

#endif //__STAR_SDK_MSG_ISF_PACKET_H
