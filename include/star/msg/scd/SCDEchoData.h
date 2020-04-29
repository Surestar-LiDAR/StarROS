/**
 * @author   lucb
 * @date     2019/12/12
 */

#ifndef __STAR_SDK_FAST_ECHO_DATA_H
#define __STAR_SDK_FAST_ECHO_DATA_H

#include <star/Star.h>
#include <cstdint>
#include <star/Shot.h>

namespace ss {

class Debug;

namespace cfg {
struct DeviceConfigure;
}

namespace msg {
namespace scd {

struct __star_export EchoDataSimplified {
    uint16_t data_id;
    uint32_t t0stamp;
    uint32_t scan_count;
    uint32_t turn_count;
    uint32_t rise[4];
    uint16_t peak[4];
    bool     mpia_sel;
    bool     hl_sel[4];
    bool     sl;

#if 0
    deEchoMsg_SIMPLIFIED_S toEchoMsg() const noexcept;
#endif

    LasShot_S toLasShot(
            uint32_t scdVersion,
            const cfg::DeviceConfigure &device,
            Debug& debug) const noexcept;
};

struct __star_export EchoData {
    struct Rise {
        uint32_t high;
        uint32_t low;
    };

    struct Peak {
        uint16_t high;
        uint16_t low;
    };

    uint16_t data_id;
    uint32_t t0stamp;
    uint32_t scan_count;
    uint32_t turn_count;
    Rise     rise[4];
    Peak     peak[4];
    bool     mpia_sel;
    bool     hl_sel[4];
    bool     sl;

#if 0
    deEchoMsg_S toEchoMsg() const noexcept;
#endif
	
    LasShot_S toLasShot(
            uint32_t scdVersion,
            const cfg::DeviceConfigure &device,
            bool lastSL,
            Debug& debug
    ) const noexcept ;
};

class EchoDataSimplifiedMessage {
public:
    typedef EchoDataSimplified fast_data;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& ) = 0;
};

class EchoDataMessage {
public:
    typedef EchoData fast_data;
    virtual fast_data get() const = 0;
    virtual void set(const fast_data& ) = 0;
};

}


}
}


#endif //__STAR_SDK_FAST_ECHO_DATA_H
