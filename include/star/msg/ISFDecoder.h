/**
 * @author   lucb
 * @date     2019/12/24
 */

#ifndef __STAR_SDK_MSG_ISF_DECODER_H
#define __STAR_SDK_MSG_ISF_DECODER_H

#include <star/Star.h>

#include <star/msg/StreamDecoder.h>

namespace ss {
namespace msg {

class __star_export ISFDecoder : public StreamDecoder {
public:
    enum Version  {
        ISF_VER_4   = 400,
        ISF_VER_4_1 = 410,
        ISF_VER_5   = 500
    };
    explicit ISFDecoder(Version version);

    Version version() const;

protected:
    const Message* decodeMessage() override;
    bool writeRecordHeader(Stream* stream) override;

private:
    Version _version;
    uint8_t _lastDataGrade;
};

}
}


#endif //__STAR_SDK_MSG_ISF_DECODER_H
