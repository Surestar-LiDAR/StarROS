/**
 * @author   lucb
 * @date     2020/1/7
 */

#ifndef _STAR_SDK_MSG_IMP_STORAGE_H
#define _STAR_SDK_MSG_IMP_STORAGE_H

#include <star/Star.h>
#include <star/Storage.h>
#include <star/msg/scd/v3/Metadata.h>
#include <star/utils/mutex.h>

#include <cstdio>

namespace ss {
namespace msg {

class __star_export IMPV3Storage : public ss::Storage {
public:
    IMPV3Storage();

    ~IMPV3Storage();

    bool open() override;

    bool close() override;

	bool writeMetaData(const void* metaData, std::size_t length) override;
	
    bool writeFastData(const void* fastData, std::size_t length) override;

    bool writeSlowData(const void* slowData, std::size_t length) override;

    bool openNewFile();

protected:
    bool cleanAndClose();

private:
    std::recursive_mutex _mutex;
    FILE*                _file;
    long                 _meta_offset;
    long                 _fast_offset;
    long                 _slow_offset;
};

}
}

#endif //_STAR_SDK_MSG_IMP_STORAGE_H
