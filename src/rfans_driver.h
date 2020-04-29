/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _RFANS_DRIVER_H_
#define _RFANS_DRIVER_H_
#include <ros/ros.h>
#include "ioapi.h"
#include <stdint.h>
#include <star/Reader.h>
#include <star/Export.h>
#include <star/calc/CalCoor.h>
#include <star/calc/Interpolation.h>
#include "star/Pcap.h"
#include"star/IOFile.h"
#include <star/IOFile.h>
#include <star/Socket.h>
#include <dynamic_reconfigure/server.h>
#include "rfans_driver/FilterParamsConfig.h"

using namespace ss;
using namespace ss::calc;

namespace rfans_driver
{
void openReader(Reader& reader, const std::string& filePath, Configure& configure);
void openConfigure(Configure& configure, const std::string& cfg, const std::string& revise);
void openTcpServer(Reader& reader, const std::string& ip, uint16_t port, Configure& configure);

static int out_count=0;
extern vector<RFANS_XYZ_S> temple_1;
class Rfans_Driver
{
public:
    Rfans_Driver(ros::NodeHandle node, ros::NodeHandle nh);
    ~Rfans_Driver();
    int spinOnce();
    int prog_Set(DEB_PROGRM_S &program);
    int datalevel_Set(DEB_PROGRM_S &program);
    rfans_driver::IOAPI* getDevInstance();
    void configDeviceParams();
private:
    double calcReplayPacketRate();
    void setupNodeParams(ros::NodeHandle node,ros::NodeHandle nh);
    void InitPointcloud2(sensor_msgs::PointCloud2 &initCloud);
    void calout_clound(SHOTS_CALCOUT_S &calout,RFANS_XYZ_S &out);
    void calout_fansxyz( SHOTS_CALCOUT_S& INPUT);
    void fansxyz_clound(sensor_msgs::PointCloud2 &outCloud_sdk);
    int syn_date();
    int Asyn_date();

private:
    struct
    {
        std::string command_path;
        std::string advertise_path;
        std::string device_ip;
        std::string device_name;
        std::string simu_filepath;
        std::string  export_path;
        int dataport;
        int scnSpeed;
        //int data_level;
        bool dual_echo;
        double angle_range;
        std::string cfg_path;
        bool read_once;
        double repeat_delay_;
        bool read_fast;
        bool save_xyz;
        bool use_gps;
    } config_;
    rfans_driver::IOAPI *m_devapi;
    rfans_driver::InputPCAP *input_;

    Configure configure;
    ros::ServiceServer server_ ;
    sensor_msgs::PointCloud2 clound;
    Interpolation interpolation;
    CCalCoor calCoor;
    Export exports;
    double PonintFrequency;
    int LED_num;
    int scanSpeed;
    bool flag_simu;
    //ros::Rate packet_rate;
};

}

#endif //_RFANS_DRIVER_H_
