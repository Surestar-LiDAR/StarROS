/**
 * @author   lucb
 * @date     2020/1/7
 */

#ifndef __STAR_SDK_STORAGE_H
#define __STAR_SDK_STORAGE_H

#include <star/Star.h>

#include <string>
#include <cstdint>
#include <cstddef>

namespace ss {

namespace msg {
struct Imp_MetaData_S;
}

class __star_export Storage {
public:
    Storage();
    virtual ~Storage();

    virtual bool open() = 0;
    virtual bool close() = 0;

    virtual void setPath(const std::string& path);

    void setFileSizeLimit(std::size_t fileSizeLimit);
    void setFileCountLimit(std::size_t fileCountLimit);

    virtual bool writeMetaData(const void* metadata, std::size_t length) = 0;
    virtual bool writeFastData(const void* fastData, std::size_t length) = 0;
    virtual bool writeSlowData(const void* slowData, std::size_t length) = 0;

    virtual FILE* openNewFile(const std::string& prefix, const std::string& suffix);

    const std::string& currentFilePath() const;

protected:
    std::string _path;
    std::string _currentFilePath;
    std::size_t _fileSizeLimit;
    std::size_t _fileCountLimit;
};

}

#endif //__STAR_SDK_STORAGE_H
