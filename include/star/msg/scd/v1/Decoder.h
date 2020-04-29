/**
 * @author   lucb
 * @date     2019/12/11
 */

#ifndef __STAR_SDK_MSG_SCD_V1_DECODER_H
#define __STAR_SDK_MSG_SCD_V1_DECODER_H

#include <star/Star.h>
#include <star/msg/StreamDecoder.h>
#include <star/msg/scd/v1/Metadata.h>

namespace ss {
namespace msg {
namespace scd {
namespace v1 {

constexpr uint64_t ID_MOTOR = 0XC7C7;
constexpr uint64_t ID_RFANS = 0XFF81;
constexpr uint64_t ID_PACK = 0XA7A7;
constexpr uint64_t ID_PACK_FAST = 0XC7C7;
constexpr uint64_t ID_SIMULATE = 0XB7B7;
constexpr uint64_t ID_TOF_0ECHO = 0XFF00;
constexpr uint64_t ID_TOF_1ECHO = 0XFF01;
constexpr uint64_t ID_TOF_2ECHO = 0XFF02;
constexpr uint64_t ID_TOF_3ECHO = 0XFF03;
constexpr uint64_t ID_TOF_4ECHO = 0XFF04;

constexpr uint64_t ID_TOF_0ECHO_SIMPLIFIED = 0XFF10;
constexpr uint64_t ID_TOF_1ECHO_SIMPLIFIED = 0XFF11;
constexpr uint64_t ID_TOF_2ECHO_SIMPLIFIED = 0XFF12;
constexpr uint64_t ID_TOF_3ECHO_SIMPLIFIED = 0XFF13;
constexpr uint64_t ID_TOF_4ECHO_SIMPLIFIED = 0XFF14;

constexpr uint64_t ID_TOF_0ECHO_AVERAGE = 0XFF20;
constexpr uint64_t ID_TOF_1ECHO_AVERAGE = 0XFF21;

constexpr uint64_t ID_PACK_DUMP = 0XFA05;
constexpr uint64_t ID_SCAN_CAMERA = 0XF906;
constexpr uint64_t ID_SCAN_CAMERA0 = 0XFD20;
constexpr uint64_t ID_SCAN_CAMERA1 = 0XFD21;
constexpr uint64_t ID_SCAN_CAMERA2 = 0XFD22;
constexpr uint64_t ID_SCAN_CAMERA3 = 0XFD23;
constexpr uint64_t ID_SCAN_CAMERA4 = 0XFD24;
constexpr uint64_t ID_SCAN_CAMERA5 = 0XFD25;
constexpr uint64_t ID_SCAN_CAMERA6 = 0XFD26;
constexpr uint64_t ID_SCAN_CAMERAF = 0XFD2F;
constexpr uint64_t ID_STATE_REPORT = 0XF0FF;
constexpr uint64_t ID_CONFIG_REPORT = 0XF0F1;
//constexpr uint64_t ID_POS_ECHO = 0XF807;
//constexpr uint64_t ID_SCAN_TURRET = 0XF708;
constexpr uint64_t ID_SYNCHRON = 0XE7E7;
constexpr uint64_t ID_SYNCHRON_FAST = 0XF7F7;
constexpr uint64_t ID_INTER_DUMP_S = 0XE7E7;
constexpr uint64_t ID_AMCW_ECHO = 0XF50A;
constexpr uint64_t ID_INCLT_ECHO = 0XFC30;
constexpr uint64_t ID_INCLT6FR_ECHO = 0XFC31;
constexpr uint64_t ID_ENVIR = 0XF0F0;
constexpr uint64_t ID_IMU = 0XFA51;
constexpr uint64_t ID_IMU_LC100 = 0XFA53;
constexpr uint64_t ID_IMU_IC100 = 0XFA54;
constexpr uint64_t ID_GPS = 0XF960;
constexpr uint64_t ID_DMI = 0XF00F;
constexpr uint64_t ID_DMI1 = 0XF870;
constexpr uint64_t ID_DMI2 = 0XF871;
constexpr uint64_t ID_POS2010 = 0xFA50;
constexpr uint64_t ID_GI510 = 0xFA55;
constexpr uint64_t ID_DIM_ZT = 0xF781;
constexpr uint64_t ID_CUSTOMER_DATA = 0xF780; //RW 零位信号脉冲计数ID

class __star_export Decoder : public StreamDecoder {
public:
    Decoder();

    const ss::msg::Message* decodeHeader();

protected:
    const ss::msg::Message* decodeMessage() override;
    bool writeRecordHeader(Stream* stream) override;

private:
    Metadata  _metadata;
};

}
}
}
}

#endif //__STAR_SDK_MSG_SCD_V1_DECODER_H
