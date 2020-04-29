/**
 * @author   lucb
 * @date     2020/4/2
 */

#ifndef __STAR_SDK_COMMON_H
#define __STAR_SDK_COMMON_H

#include <iostream>
#include <string>

#include <star/IOFile.h>
#include <star/Socket.h>
#include <star/Reader.h>
#include <star/Export.h>
#include <star/calc/CalCoor.h>
#include <star/calc/Interpolation.h>
#include "star/Pcap.h"

using namespace ss;
using namespace ss::calc;

//打开Reader
void openReader(Reader& reader, const std::string& filePath, Configure& configure)
{
    if (filePath.find(".isf") != std::string::npos) { //isf文件处理
        IOFile* stream = new IOFile(filePath); //创建文件Stream
        reader.setFormatVersion(Reader::ISF_VER_5); //ISF 5.0格式
        reader.setBlockMode(false); //已弃用
        stream->seek(0x14000); //跳过文件头
        reader.open(stream); //打开stream
        configure.deviceVersion().type = lidar::RFans;
    }
    else if(filePath.find(".pcap") != std::string::npos) { //pcap文件处理
        Pcap* stream = new Pcap(filePath);
        stream->filte(2014);
        stream->filte(2014); //Pcap过滤，只读取端口2014的数据
        reader.setFormatVersion(Reader::ISF_VER_4);
        reader.setBlockMode(false);
        reader.open(stream);
        configure.deviceVersion().type = lidar::RFans;
    }
    else { //IMP文件处理
        IOFile* stream = new IOFile(filePath);
        reader.setFormatVersion(Reader::IMP_VER_1);
        reader.setBlockMode(false);
        reader.open(stream);
        reader.waitMetadata(configure);
    }
}

void openTcpServer(Reader& reader, const std::string& ip, uint16_t port, Configure& configure)
{
    auto* stream = new ss::Socket(ss::Socket::tcp());
    stream->connect(ss::InternetEndpoint(ip, port));
    reader.setFormatVersion(Reader::IMP_VER_1);
    reader.setBlockMode(false);
    reader.open(stream);
//    reader.waitMetadata(configure);
}

void openConfigure(Configure& configure, const std::string& cfg, const std::string& revise)
{
    if (cfg.find(".ini") != std::string::npos) {
        configure.changeFormat(Configure::INI_FORMAT);
        configure.loadFile(cfg);
        configure.deviceVersion().type = lidar::RFans;
    }
    else {
        configure.changeFormat(Configure::COMPAT_FORMAT);
        configure.loadFile(cfg);
    }
    if (!revise.empty()) {
        Configure reviseConfigure(Configure::INI_FORMAT);
        reviseConfigure.loadFile(revise);
        configure.merge(reviseConfigure);
    }
}

#endif //__STAR_SDK_COMMON_H
