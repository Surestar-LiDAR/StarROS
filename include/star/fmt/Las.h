/**
 * @author   lucb
 * @date     2020/1/8
 */

#ifndef __STAR_SDK_FMT_LAS_H
#define __STAR_SDK_FMT_LAS_H

#include <star/Star.h>

#include <star/fmt/Point.h>
#include <star/fmt/Writer.h>
#include <cstdint>
#include <fstream>

namespace ss {
namespace fmt {

struct UUID {
    uint32_t data1;
    uint16_t data2;
    uint16_t data3;
    uint8_t  data4[8];
};

struct LasPublicHeader {
    const static char signature[4];
    uint16_t    sourceId;
    uint16_t    encoding;
    UUID        projectId;
    uint8_t     versionMajor;
    uint8_t     versionMinor;
    std::string systemId;
    std::string softwareId;
    uint16_t    dayOfYear;
    uint16_t    year;
    uint16_t    headSize;
    uint32_t    dataOffset;
    uint32_t    variableRecordCount;
    uint8_t     pointFormatId;
    uint16_t    pointSize;
    uint32_t    pointCount;
    uint32_t    returnPointCount[5];
    double      scaleX;
    double      scaleY;
    double      scaleZ;
    double      offsetX;
    double      offsetY;
    double      offsetZ;
    double      maxX;
    double      minX;
    double      maxY;
    double      minY;
    double      maxZ;
    double      minZ;
};

class __star_export LasWriter : public FileWriter {
public:

    LasWriter() noexcept;

    explicit LasWriter(const std::string& path) noexcept;
//    explicit LasWriter(std::ostream& os) noexcept;
    virtual ~LasWriter() noexcept;

    void writeHeader() override;
    void writePoints(const SHOTS_CALCOUT_S& shot) override;

    std::size_t write(const Point& point) override;
    void close() override;

    LasPublicHeader& header();
    const LasPublicHeader& header() const;
    void setHeader(const LasPublicHeader& header);

    std::string suffix() const override;

protected:
    void initHeader();
    std::size_t doWriteHeader();
    std::size_t encodeHeaderVer1_2(buffer_writer& buffer);
    std::size_t encodePointFormat0(buffer_writer& buffer, const Point& point);
    std::size_t encodePointFormat1(buffer_writer& buffer, const Point& point);
    std::size_t encodePointFormat2(buffer_writer& buffer, const Point& point);
    std::size_t encodePointFormat3(buffer_writer& buffer, const Point& point);
    std::size_t encodePointFormat4(buffer_writer& buffer, const Point& point);

private:
    LasPublicHeader _header;
    std::ofstream   _ofs;
    std::size_t     _pointCount;
    double          _offsetX;
    double          _offsetY;
    double          _offsetZ;
};

}
}

#endif //__STAR_SDK_FMT_LAS_H
