/**
 * @author   lucb
 * @date     2020/2/14
 */

#ifndef __STAR_SDK_EXPORT_H
#define __STAR_SDK_EXPORT_H

#include <star/Star.h>

#include <star/fmt/Writer.h>

#include <list>

#include <star/Configure.h>

namespace ss {

class __star_export Export {
public:
    enum FormatType {
        LAS_FORMAT,
        XYZ_TXT_FORMAT,
        AGR_TXT_FORMAT,
        AGR_BIN_FORMAT,
        PLR_TXT_FORMAT,
        PLR_BIN_FORMAT,
        PTX_FORMAT,
        RFANS_FORMAT,
        RGP_FORMAT,
        ST_FORMAT,
        YSJ_FORMAT
    };
    Export();
    Export(const std::string& path, const std::string &fileName);
    ~Export();

    //void setup(const Configure& configure);

    /**
     * 设置保存的路径和文件名
     * @param path 文件保存的路径
     * @param fileName 不带扩展名的文件名
     */
    void setPathAndFileName(const std::string& path, const std::string& fileName);

    ss::fmt::Writer* addWriter(const Configure& configure, FormatType formatType);
    ss::fmt::Writer* addWriter(const Configure& configure, FormatType formatType, uint32_t flags);

    ss::fmt::Writer* addWriter(const Configure& configure,
            FormatType formatType, const std::string& file);
    ss::fmt::Writer* addWriter(const Configure& configure,
            FormatType formatType, const std::string& file, uint32_t flags);

    void writePoints(const SHOTS_CALCOUT_S& shot);

    void clear();
private:
    std::string                 _path;
    std::string                 _fileName;
    std::list<ss::fmt::Writer*> _writers;
};

}


#endif //__STAR_SDK_EXPORT_H
