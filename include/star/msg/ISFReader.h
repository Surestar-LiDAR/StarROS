/**
 * @author   lucb
 * @date     2019/12/25
 */

#ifndef __STAR_SDK_ISF_READER_H
#define __STAR_SDK_ISF_READER_H

#include <star/Star.h>

#include <star/msg/Reader.h>

#include <vector>

namespace ss {
namespace msg {

class __star_export ISFReader : public Reader {
public:
    ISFReader();

    bool wait_metadata() override;

    void set_metadata(const ss::Configure& configure) override;

    int64_t last_timestamp() const;
    void set_last_timestamp(int64_t last_timestamp);


private:
    int64_t _last_timestamp;
};

}
}


#endif //__STAR_SDK_ISF_READER_H
