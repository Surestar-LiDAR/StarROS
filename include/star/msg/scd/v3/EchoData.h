/**
 * @author   lucb
 * @date     2019/12/9
 */

#ifndef __STAR_SDK_MSG_SCD_VER3_ECHO_DATA_H
#define __STAR_SDK_MSG_SCD_VER3_ECHO_DATA_H

#include <star/Star.h>

#include <star/msg/Message.h>
#include <star/msg/scd/SCDMessage.h>
#include <star/msg/scd/SCDEchoData.h>

#include <cstdint>

namespace ss {
namespace msg {
namespace scd {
namespace v3 {

#pragma pack(push, 2)

struct _TOF0ECHO_S {
    uint16_t dataId : 16;//0
    uint16_t t0stampL : 16;
    uint16_t t0stampH : 16;
    uint16_t scanContH : 16;
    uint16_t scanContL: 4;
    uint16_t turnCountH : 12;
    uint16_t mpia_set : 1;
    uint16_t HL_set1 : 1;
    uint16_t HL_set2 : 1;
    uint16_t HL_set3 : 1;
    uint16_t HL_set4 : 1;
    uint16_t reserve4 : 1;
    uint16_t turnCountL : 10;
} ;

/************TOF One Echo********************************
*record as follows
*C、T0_Time_Stamp、
*Rise1、Fall1、Peak1
*Mirpos_Integer、Period才Residue
**********************************************************/
struct _TOF1ECHO_S {
    uint16_t dataId : 16; //1
    uint16_t t0stampL : 16;
    uint16_t t0stampH : 16;
    uint16_t Hrise1H : 16;
    uint16_t Hrise1L : 8;
    uint16_t Lrise1H : 8;
    uint16_t Lrise1L : 16;
    uint16_t Hpeak1:  12;
    uint16_t mfStatusL : 4;
    uint16_t Lpeak1:  12;
    uint16_t mfStatusH : 4;
    uint16_t fireValStatus : 16;
    uint16_t scanContH : 16;
    uint16_t scanContL : 4;
    uint16_t turnCountH : 12;
    uint16_t mpia_set : 1;
    uint16_t HL_set1 : 1;
    uint16_t HL_set2 : 1;
    uint16_t HL_set3 : 1;
    uint16_t HL_set4 : 1;
    uint16_t reserve4 : 1;
    uint16_t turnCountL : 10;
};

/*************TOF Two Echo*******************************
*record as follows
*C、T0_Time_Stamp、
*Rise1、Rise2、
*Fall1、Fall2、
*Peak1、Peak2、
*Mirpos_Integer、Period才Residue
**********************************************************/
struct _TOF2ECHO_S {
    uint16_t dataId : 16;//2
    uint16_t t0stampL : 16;
    uint16_t t0stampH : 16;
    uint16_t Hrise1H : 16;
    uint16_t Hrise1L : 6;
    uint16_t Lrise1H : 10;
    uint16_t Lrise1L : 12;
    uint16_t Hpeak1H : 4;
    uint16_t Hpeak1L : 8;
    uint16_t Lpeak1H : 8;
    uint16_t Lpeak1L : 4;
    uint16_t Hpeak2 : 12;
    uint16_t Lpeak2 : 12;
    uint16_t reserve1 : 4;
    uint16_t diff_Hrise2 : 16;
    uint16_t diff_Lrise2 : 16;
    uint16_t fireValStatus : 16;
    uint16_t mfStatus : 8;
    uint16_t reserve2 : 8;
    uint16_t scanContH : 16;
    uint16_t scanContL : 4;
    uint16_t turnCountH : 12;
    uint16_t mpia_set : 1;
    uint16_t HL_set1 : 1;
    uint16_t HL_set2 : 1;
    uint16_t HL_set3 : 1;
    uint16_t HL_set4 : 1;
    uint16_t reserve4 : 1;
    uint16_t turnCountL : 10;
};

/***************TOF Three Echo*****************************
*record as follows
*C、T0_Time_Stamp、
*Rise1、Rise2、Rise3、
*Fall1、Fall2、Fall3、
*Peak1、Peak2、Peak3、
*Mirpos_Integer、Period才Residue
**********************************************************/
struct _TOF3ECHO_S {
    uint16_t dataId : 16;
    uint16_t t0stampL : 16;
    uint16_t t0stampH : 16;
    uint16_t Hrise1H : 16;
    uint16_t Hrise1L : 6;
    uint16_t Lrise1H : 10;
    uint16_t Lrise1L : 12;
    uint16_t Hpeak1H : 4;
    uint16_t Hpeak1L : 8;
    uint16_t Lpeak1H : 8;
    uint16_t Lpeak1L : 4;
    uint16_t Hpeak2 : 12;
    uint16_t Lpeak2 : 12;
    uint16_t mfStatusL : 4;
    uint16_t Hpeak3 : 12;
    uint16_t mfStatusH : 4;
    uint16_t diff_Hrise2 : 16;
    uint16_t diff_Lrise2 : 16;
    uint16_t diff_Hrise3 : 16;
    uint16_t diff_Lrise3 : 16;
    uint16_t fireValStatus : 16;
    uint16_t scanContH : 16;
    uint16_t scanContL : 4;
    uint16_t turnCountH : 12;
    uint16_t mpia_set : 1;
    uint16_t HL_set1 : 1;
    uint16_t HL_set2 : 1;
    uint16_t HL_set3 : 1;
    uint16_t HL_set4 : 1;
    uint16_t reserve4 : 1;
    uint16_t turnCountL : 10;
};

struct _TOF4ECHO_S {
    uint16_t dataId : 16;
    uint16_t t0stampL : 16;
    uint16_t t0stampH : 16;
    uint16_t Hrise1H : 16;
    uint16_t Hrise1L : 6;
    uint16_t Lrise1H : 10;
    uint16_t Lrise1L : 12;
    uint16_t Hpeak1H : 4;
    uint16_t Hpeak1L: 8;
    uint16_t Lpeak1H : 8;
    uint16_t Lpeak1L : 4;
    uint16_t Hpeak2 : 12;
    uint16_t Lpeak2 : 12;
    uint16_t mfStatusL : 4;
    uint16_t Hpeak3 : 12;
    uint16_t mfStatusH : 4;
    uint16_t Hpeak4 : 12;
    uint16_t reserve3 : 4;
    uint16_t diff_Hrise2 : 16;
    uint16_t diff_Lrise2 : 16;
    uint16_t diff_Hrise3 : 16;
    uint16_t diff_Lrise3 : 16;
    uint16_t diff_Hrise4 : 16;
    uint16_t diff_Lrise4 : 16;
    uint16_t fireValStatus : 16;
    uint16_t scanContH : 16;
    uint16_t scanContL : 4;
    uint16_t turnCountH : 12;
    uint16_t mpia_set : 1;
    uint16_t HL_set1 : 1;
    uint16_t HL_set2 : 1;
    uint16_t HL_set3 : 1;
    uint16_t HL_set4 : 1;
    uint16_t reserve4 : 1;
    uint16_t turnCountL : 10;
};

#pragma pack(pop)

template<typename _DataType>
class EchoDataSimplifiedMessage :
        public scd::Message<_DataType>,
        public scd::EchoDataSimplifiedMessage {

};

template<typename _DataType>
class EchoDataMessage :
        public scd::Message<_DataType>,
        public scd::EchoDataMessage {

};

class __star_export EchoZeroData :
        public EchoDataMessage<_TOF0ECHO_S> {
public:
    inline uint64_t dataId() const override
    {
        return this->data().dataId;
    }

    fast_data get() const;

    void set(const fast_data& data);
};

class __star_export EchoOneData :
        public EchoDataMessage<_TOF1ECHO_S> {
public:
    inline uint64_t dataId() const override
    {
        return this->data().dataId;
    }

    fast_data get() const;

    void set(const fast_data& data);
};

class __star_export EchoTwoData :
        public EchoDataMessage<_TOF2ECHO_S> {
public:
    inline uint64_t dataId() const override
    {
        return this->data().dataId;
    }

    fast_data get() const;

    void set(const fast_data& data);
};

class __star_export EchoThreeData :
        public EchoDataMessage<_TOF3ECHO_S> {
public:
    inline uint64_t dataId() const override
    {
        return this->data().dataId;
    }

    fast_data get() const;

    void set(const fast_data& data);
};

class __star_export EchoFourData :
        public EchoDataMessage<_TOF4ECHO_S> {
public:
    inline uint64_t dataId() const override
    {
        return this->data().dataId;
    }

    fast_data get() const;

    void set(const fast_data& data);
};

}
}
}
}

#endif //__STAR_SDK_MSG_SCD_VER3_ECHO_DATA_H
