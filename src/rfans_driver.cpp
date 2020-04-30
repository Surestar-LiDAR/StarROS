/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include "rfans_driver.h"
#include "rfans_driver/RfansCommand.h"
#include "rfans_driver/RfansScan.h"

sensor_msgs::PointCloud2 outCloud_sdk ;
extern ros::Publisher  sdk_output;
extern double min_range;
extern double max_range;
extern double min_angle;
extern double max_angle;
extern int ringID;
extern bool use_laserSelection_;
extern bool distortion_flag;
extern float Angle_resolution;
extern bool Device;
namespace rfans_driver {
vector<RFANS_XYZ_S> temple_1;
static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static const int RFANS_PACKET_NUM = 1024 ;
size_t packet_size_pcap = 1206;
Reader reader;
Interpolation interpolation;
CCalCoor calCoor;
static Rfans_Driver *s_this = NULL;

bool CommandHandle(rfans_driver::RfansCommand::Request  &req,
                   rfans_driver::RfansCommand::Response &res)
{
  res.status = 1;

  ROS_INFO("request: cmd= %d , speed = %d Hz", (int)req.cmd, (int)req.speed);
  ROS_INFO("sending back response: [%d]", (int)res.status);

  DEB_PROGRM_S tmpProg ;

  unsigned int tmpData = 0;
  tmpProg.cmdstat = (DEB_CMD_E)req.cmd;
  tmpProg.dataFormat = eFormatCalcData;
  tmpProg.scnSpeed = req.speed;
  tmpProg.dataLevel = (int)req.use_double_echo;
  if(tmpProg.dataLevel == 0){
    tmpData = CMD_LEVEL3_ECHO;
  } else {
    tmpData = CMD_LEVEL3_DUAL_ECHO;
  }
  if(s_this) {
    s_this->prog_Set(tmpProg);
    s_this->getDevInstance()->HW_WRREG(0, REG_DATA_LEVEL_OLD, tmpData);
    s_this->getDevInstance()->HW_WRREG(0, REG_DATA_LEVEL, tmpData);
  }
  return true;
}
void openReader(Reader& reader, const std::string& filePath, Configure& configure)
{
  if (filePath.find(".isf") != std::string::npos) {
    IOFile* stream = new IOFile(filePath);
    reader.setFormatVersion(Reader::ISF_VER_5);
    reader.setBlockMode(false);
    stream->seek(0x14000);
    reader.open(stream);

  }
  else if(filePath.find(".pcap") != std::string::npos) {
    Pcap* stream = new Pcap(filePath);
    stream->filte(2014);
    reader.setFormatVersion(Reader::ISF_VER_5);
    reader.setBlockMode(false);
    reader.open(stream);
  }
  else {
    IOFile* stream = new IOFile(filePath);
    reader.setFormatVersion(Reader::IMP_VER_1);
    reader.setBlockMode(false);
    reader.open(stream);
    reader.waitMetadata(configure);
  }

}
void openConfigure(Configure& configure, const std::string& cfg, const std::string& revise)
{

  if (cfg.find(".ini") != std::string::npos) {
    configure.changeFormat(Configure::INI_FORMAT);
    configure.loadFile(cfg);
    configure.deviceVersion().type = lidar::RFans;
    //ROS_INFO("lidar::CFans");
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

void openTcpServer(Reader& reader, const std::string& ip, uint16_t port, Configure& configure)
{
  auto* stream = new ss::Socket(ss::Socket::udp());
  stream->bind(InternetEndpoint(port));
  reader.setFormatVersion(Reader::ISF_VER_5);
  reader.setBlockMode(false);
  reader.open(stream);
}

Rfans_Driver::Rfans_Driver(ros::NodeHandle node, ros::NodeHandle nh ):configure(Configure::COMPAT_FORMAT)
{


  setupNodeParams(node,nh);
  server_ = node.advertiseService("rfans_driver/" + config_.command_path, CommandHandle);
  if((config_.device_name=="C-Fans-128")||(config_.device_name=="C-Fans-32"))
  {
    openConfigure(configure, config_.cfg_path, "");
  }
  else{
    configure.deviceVersion().type = lidar::RFans;
  }


  scanSpeed=config_.scnSpeed;
  exports.setPathAndFileName(config_.export_path, "data");
  exports.addWriter(configure, Export::XYZ_TXT_FORMAT);
  //packet_rate = calcReplayPacketRate();

  if (config_.simu_filepath != "") {
    openReader(reader, config_.simu_filepath, configure);
    flag_simu=0;
  } else {
    openTcpServer(reader, config_.device_ip, config_.dataport, configure);
    m_devapi = new rfans_driver::IOSocketAPI(config_.device_ip, config_.dataport, config_.dataport);
    configDeviceParams();
    flag_simu=1;
  }
  InitPointcloud2(outCloud_sdk);


  calCoor.setup(configure, &interpolation);
  calCoor.multiLevelRegister().setDefaultReviseOptions(3);
  auto options = calCoor.multiLevelRegister().reviseOptions();
  calCoor.multiLevelRegister().setReviseOptions(options);


  s_this = this ;

}

Rfans_Driver::~Rfans_Driver()
{

}
void Rfans_Driver::calout_fansxyz( SHOTS_CALCOUT_S& INPUT)
{

  for (std::size_t idx = 0; idx < 4; ++idx) {
    if (INPUT.lasShot.m_pulse[idx].flag) {
      //ROS_INFO("out_count=%d",out_count);
      ros::spinOnce();
      if(INPUT.lasShot.m_pulse[idx].m_fRange > max_range || INPUT.lasShot.m_pulse[idx].m_fRange < min_range
         || INPUT.lasShot.m_dAngleX>max_angle || INPUT.lasShot.m_dAngleX<min_angle)
      {
        continue;
      }
      //ROS_INFO("INPUT.lasShot.m_dAngleX=%f",INPUT.lasShot.m_dAngleX);
      temple_1.resize(out_count+1);
      if(use_laserSelection_){
        if(INPUT.lasShot.lidarID != ringID)
        {
          continue;
        }
        else {
          temple_1[out_count].x=INPUT.m_pulse[idx].X;
          temple_1[out_count].y=INPUT.m_pulse[idx].Y;
          temple_1[out_count].z=INPUT.m_pulse[idx].Z;
          temple_1[out_count].range=INPUT.lasShot.m_pulse[idx].m_fRange;
          temple_1[out_count].intent=INPUT.lasShot.m_pulse[idx].m_uInt;
          temple_1[out_count].hangle=INPUT.lasShot.m_dAngleX;
          temple_1[out_count].laserid=INPUT.lasShot.lidarID;
          temple_1[out_count].timeflag=INPUT.m_dGPSTime;
          temple_1[out_count].mirrorid=INPUT.lasShot.mirrorNumb;
          temple_1[out_count].col=temple_1[out_count].hangle/(scanSpeed*LED_num*360/PonintFrequency);
          ++out_count;
          continue;
        }
      }
      temple_1[out_count].x=INPUT.m_pulse[idx].X;

      temple_1[out_count].y=INPUT.m_pulse[idx].Y;
      temple_1[out_count].z=INPUT.m_pulse[idx].Z;

      temple_1[out_count].range=INPUT.lasShot.m_pulse[idx].m_fRange;
      temple_1[out_count].intent=INPUT.lasShot.m_pulse[idx].m_uInt;
      temple_1[out_count].hangle=INPUT.lasShot.m_dAngleX;
      temple_1[out_count].laserid=INPUT.lasShot.lidarID;
      if(config_.use_gps)
      {temple_1[out_count].timeflag=INPUT.m_dGPSTime;}
      else{
        temple_1[out_count].timeflag=INPUT.lasShot.utcTime;
      }

      temple_1[out_count].mirrorid=INPUT.lasShot.mirrorNumb;
      double a;
      a=scanSpeed*LED_num*360;
      double b=a/PonintFrequency;
      double c=scanSpeed*LED_num*360/PonintFrequency;
      temple_1[out_count].col=temple_1[out_count].hangle/(scanSpeed*LED_num*360/PonintFrequency);
      ++out_count;

    }
  }

}

void Rfans_Driver::fansxyz_clound(sensor_msgs::PointCloud2 &outCloud_sdk)
{

  outCloud_sdk.header.stamp = ros::Time::now();
  outCloud_sdk.width = out_count;
  outCloud_sdk.data.resize( outCloud_sdk.point_step*outCloud_sdk.width);
  outCloud_sdk.row_step = outCloud_sdk.data.size();
  memcpy(&outCloud_sdk.data[0] , &temple_1[0], outCloud_sdk.data.size() );
}

int Rfans_Driver::Asyn_date()
{
  int ret = 1;
  int write_flag=0;
  vector<double> packet_time;
  vector<int> packet_size;
  int flag=1;
  ros::Rate packet_rate=calcReplayPacketRate()/100;

  while (!reader.eof()||ret)
  {
      //ROS_INFO("li");

     //ros::Time t1=ros::Time::now();
     if (config_.read_fast == false)
       packet_rate.sleep();
     //ros::Time t2=ros::Time::now();

    auto  packets = reader.getPackets();
    //ROS_INFO("packet_size=%d",packets.points.size());
    if(packets.points.size()==0)
    {
      flag=8;//the first data of CFAN-128 is 0;
    }
    for (auto iter = packets.fast_synchrons.begin(); iter != packets.fast_synchrons.end(); ++iter) {
      interpolation.updatePpsSign(*iter, true);
    }
    for (auto iter = packets.slow_synchrons.begin(); iter != packets.slow_synchrons.end(); ++iter) {
      interpolation.updatePpsSign(*iter, false);
    }

    if (flag<=6)
    {
      packet_size.push_back(packets.points.size());
      //ROS_INFO("packet_size=%x",packets.points.at(20).dataID);
      packet_time.push_back(packets.points.at(0).utcTime);
      if(packet_time.size()==6)
      {
        PonintFrequency=(packet_size.at(5))/(packet_time.at(5)-packet_time.at(4));

        if(packets.points.at(20).dataID==0x50 ||packets.points.at(20).dataID==0x59 || packets.points.at(20).dataID == 0x5A||packets.points.at(20).dataID==0x27||packets.points.at(20).dataID==0x27||packets.points.at(20).dataID==0x28
           ||packets.points.at(20).dataID==0x29||packets.points.at(20).dataID==0x5E||packets.points.at(20).dataID==0x5F||packets.points.at(20).dataID==0x60||packets.points.at(20).dataID==0x33||packets.points.at(20).dataID==0x45
           ||packets.points.at(20).dataID==0x40||packets.points.at(20).dataID==0x4A||packets.points.at(20).dataID==0x83)
        {
          LED_num=32;
        }
        else if (packets.points.at(20).dataID==0x57 ||packets.points.at(20).dataID==0x58 || packets.points.at(20).dataID == 0x5B||packets.points.at(20).dataID==0x5C||packets.points.at(20).dataID==0x22||packets.points.at(20).dataID==0x24
                 ||packets.points.at(20).dataID==0x32||packets.points.at(20).dataID==0x41||packets.points.at(20).dataID==0x42||packets.points.at(20).dataID==0x43||packets.points.at(20).dataID==0x44||packets.points.at(20).dataID==0x46
                 ||packets.points.at(20).dataID==0x47||packets.points.at(20).dataID==0x48||packets.points.at(20).dataID==0x49||packets.points.at(20).dataID==0x4B||packets.points.at(20).dataID==0x4C||packets.points.at(20).dataID==0x4D
                 ||packets.points.at(20).dataID==0x4E||packets.points.at(20).dataID==0x4F||packets.points.at(20).dataID==0x90)
        {
          LED_num=16;

        }
        else if(packets.points.at(20).dataID==0x80||packets.points.at(20).dataID==0x81 )
        {
          LED_num=8;
        }
        else {
          ROS_INFO("no matching device model");
          LED_num=32;

          ///break;
        }
      }
      flag++;
    }
    for (auto iter = packets.points.begin(); iter != packets.points.end(); ++iter) {
     SHOTS_CALCOUT_S calout;
      calout.lasShot = *iter;
      calCoor.calcXYZ(&calout);
      if(config_.save_xyz)
      {
        if(!write_flag)
        {
          exports.writePoints(calout);
        }
      }


      calout_fansxyz( calout);


    }
    fansxyz_clound(outCloud_sdk);


    sdk_output.publish(outCloud_sdk);
    out_count=0;
    if(reader.eof())
    {
      if(config_.read_once)
      {
        ret=0;
      }
      else{
        ret=1;
        write_flag=1;
        openReader(reader, config_.simu_filepath, configure);
         usleep(rint(config_.repeat_delay_ * 1000000.0));
      }

    }
    else {
      ret=0;
    }
  }
  return ret;

}
int Rfans_Driver::syn_date()
{

  int ret = 1;
  vector<double> packet_time;
  vector<int> packet_size;
  int flag=0;
  while (!reader.eof())
  {

    auto  packets = reader.getPackets();
    if(packets.points.size()==0)
    {
		flag=8;//the first data of CFAN-128 is 0;
    }
    for (auto iter = packets.fast_synchrons.begin(); iter != packets.fast_synchrons.end(); ++iter) {
      interpolation.updatePpsSign(*iter, true);
    }
    for (auto iter = packets.slow_synchrons.begin(); iter != packets.slow_synchrons.end(); ++iter) {
      interpolation.updatePpsSign(*iter, false);
    }

    if (flag<=6)
    {
      if(packets.points.size()>0)
      {
        packet_size.push_back(packets.points.size());
        packet_time.push_back(packets.points.at(0).utcTime);
        if(packet_time.size()==6)
        {
          PonintFrequency=(packet_size.at(5))/(packet_time.at(5)-packet_time.at(4));
          if(packets.points.at(20).dataID==0x50 ||packets.points.at(20).dataID==0x59 || packets.points.at(20).dataID == 0x5A||packets.points.at(20).dataID==0x27||packets.points.at(20).dataID==0x27||packets.points.at(20).dataID==0x28
             ||packets.points.at(20).dataID==0x29||packets.points.at(20).dataID==0x5E||packets.points.at(20).dataID==0x5F||packets.points.at(20).dataID==0x60||packets.points.at(20).dataID==0x33||packets.points.at(20).dataID==0x45
             ||packets.points.at(20).dataID==0x40||packets.points.at(20).dataID==0x4A)
          {
            LED_num=32;
          }
          else if (packets.points.at(20).dataID==0x57 ||packets.points.at(20).dataID==0x58 || packets.points.at(20).dataID == 0x5B||packets.points.at(20).dataID==0x5C||packets.points.at(20).dataID==0x22||packets.points.at(20).dataID==0x24
                   ||packets.points.at(20).dataID==0x32||packets.points.at(20).dataID==0x41||packets.points.at(20).dataID==0x42||packets.points.at(20).dataID==0x43||packets.points.at(20).dataID==0x44||packets.points.at(20).dataID==0x46
                   ||packets.points.at(20).dataID==0x47||packets.points.at(20).dataID==0x48||packets.points.at(20).dataID==0x49||packets.points.at(20).dataID==0x4B||packets.points.at(20).dataID==0x4C||packets.points.at(20).dataID==0x4D
                   ||packets.points.at(20).dataID==0x4E||packets.points.at(20).dataID==0x4F||packets.points.at(20).dataID==0x90)
          {
            LED_num=16;
          }
          else if(packets.points.at(20).dataID==0x80||packets.points.at(20).dataID==0x81 )
          {
            LED_num=8;
          }
          else {

             LED_num=32;
            ROS_INFO("no matching device model");
          }

        }
        flag++;

      }
    }

    for (auto iter = packets.points.begin(); iter != packets.points.end(); ++iter) {
      SHOTS_CALCOUT_S calout;
      calout.lasShot = *iter;
      calCoor.calcXYZ(&calout);

      if(config_.save_xyz)
      {
        exports.writePoints(calout);
      }

      calout_fansxyz( calout);
    }
    fansxyz_clound(outCloud_sdk);
    if (config_.repeat_delay_ > 0.0)
    {
      usleep(rint(config_.repeat_delay_ * 1000000.0));
    }
    sdk_output.publish(outCloud_sdk);
    out_count=0;
  ros::spinOnce();
  }
  return ret;

}

/** @brief Rfnas Driver Core */
int Rfans_Driver::spinOnce()
{
  bool dec;
  if(flag_simu)
  {
    dec=syn_date();//synchronous;

  }
  else {
    dec=Asyn_date();//Asynchronous;
  }
//ROS_INFO("CommandHandle1");
  return dec;
}



void Rfans_Driver::InitPointcloud2(sensor_msgs::PointCloud2 &initCloud)
{

  static const size_t DataSize = 0;
  initCloud.data.clear();
  initCloud.data.resize( DataSize);
  initCloud.is_bigendian = false ;
  initCloud.fields.resize(11);
  initCloud.is_dense = false;

  int tmpOffset = 0 ;
  for(int i=0; i < initCloud.fields.size() ;i++) {
    switch(i) {
    case 0:
      initCloud.fields[i].name = "x" ;
      initCloud.fields[i].datatype = 7u;
      break;
    case 1:
      initCloud.fields[i].name = "y" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 2:
      initCloud.fields[i].name = "z" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 3:
      initCloud.fields[i].name = "intensity" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 4:
      initCloud.fields[i].name = "laserid" ;
      initCloud.fields[i].datatype = 5u;
      tmpOffset += 4;
      break;
    case 5:
      initCloud.fields[i].name = "timeflag" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 6:
      initCloud.fields[i].name = "hangle" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 7:
      initCloud.fields[i].name = "pulseWidth";
      initCloud.fields[i].datatype = 7u;
      tmpOffset +=4;
      break;
    case 8:
      initCloud.fields[i].name = "range";
      initCloud.fields[i].datatype = 7u;
      tmpOffset +=4;
      break;
    case 9:
      initCloud.fields[i].name = "rol";
      initCloud.fields[i].datatype = 5u;
      tmpOffset +=4;
      break;
    case 10:
      initCloud.fields[i].name = "mirrorid" ;
      initCloud.fields[i].datatype = 2u;
      tmpOffset += 4;
      break;
    }
    initCloud.fields[i].offset = tmpOffset ;
    initCloud.fields[i].count = 1 ;
  }
  initCloud.height = 1;
  initCloud.point_step = sizeof(RFANS_XYZ_S);
  initCloud.row_step = DataSize ;
  initCloud.width = 0 ;
  std::string node_name = ros::this_node::getName();

  std::string frame_id_str = "/world";
  std::string frame_id_path = node_name + "/frame_id";
  std::string ip_str = std::string("rfans_driver/") + "device_ip";
  std::string data_level_param = std::string("rfans_driver/")+"data_level";
  ros::param::get(frame_id_path,frame_id_str);
  initCloud.header.frame_id = frame_id_str;



}

void Rfans_Driver::setupNodeParams(ros::NodeHandle node,ros::NodeHandle nh)
{
  node.param<std::string>("model",config_.device_name,"R-Fans-32");

  if((config_.device_name=="R-Fans-32")||(config_.device_name=="R-Fans-16"))
  {
    Device=true;

  }
  else{
    Device=false;}




  nh.param<bool>("read_fast", config_.read_fast, false);
  nh.param<std::string>("advertise_name", config_.advertise_path, "rfans_packets");
  nh.param<std::string>("control_name", config_.command_path, "rfans_control");
  nh.param<int>("device_port", config_.dataport, 2014);
  nh.param<bool>("save_xyz", config_.save_xyz, false);
  nh.param<bool>("use_gps", config_.use_gps, true);
  nh.param<std::string>("device_ip", config_.device_ip, "192.168.0.3");
  nh.param<int>("rps", config_.scnSpeed, 10);

  nh.param<std::string>("readfile_path", config_.simu_filepath, "");
  nh.param<std::string>("cfg_path", config_.cfg_path, "");
  nh.param<bool>("use_double_echo", config_.dual_echo, "false");
  //nh.param<int>("data_level", config_.data_level, 0);
  nh.param<double>("cut_angle_range",config_.angle_range,360.0);
  nh.param<std::string>("OutExport_path",config_.export_path,"/home/bkth/qt_ws/src/ROSDriver/out");
  nh.param<bool>("read_once",config_.read_once,false);
  nh.param<double>("repeat_delay",config_.repeat_delay_,0.0);
}


rfans_driver::IOAPI* Rfans_Driver::getDevInstance(){
  return m_devapi;
}
/** @brief control the device
     *  @param .parameters
     */
int Rfans_Driver::prog_Set(DEB_PROGRM_S &program)
{
  unsigned int tmpData = 0;

  switch (program.dataFormat) {
  case eFormatCalcData:
    tmpData |= CMD_CALC_DATA;
    break;
  case eFormatDebugData:
    tmpData |= CMD_DEBUG_DATA;
    break;
  }
  m_devapi->HW_WRREG(0, REG_DATA_TRANS, tmpData);
  m_devapi->HW_WRREG(0, REG_DATA_TRANS_OLD, tmpData);
  //===============================================================
  tmpData = 0;
  switch (program.scnSpeed) {
  case ANGLE_SPEED_10HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_10HZ;
    break;
  case ANGLE_SPEED_20HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_20HZ;
    break;
  case ANGLE_SPEED_5HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_5HZ;
    break;
  default:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_5HZ;
    break;
  }

  tmpData |= CMD_LASER_ENABLE;
  switch (program.cmdstat) {
  case eDevCmdWork:
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL_OLD, tmpData);
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case eDevCmdIdle:
    tmpData = CMD_RCV_CLOSE;
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL_OLD, tmpData);
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case eDevCmdAsk:
    break;
  default:
    break;
  }

  return 0;

}

int Rfans_Driver::datalevel_Set(DEB_PROGRM_S &program)
{
  unsigned int regData =0;
  switch (program.dataFormat) {
  case eFormatCalcData:
    regData |= CMD_CALC_DATA;
    break;
  case eFormatDebugData:
    regData |= CMD_DEBUG_DATA;
    break;
  }
  m_devapi->HW_WRREG(0, REG_DATA_TRANS, regData);
  m_devapi->HW_WRREG(0, REG_DATA_TRANS_OLD, regData);
  regData =0;
  switch (program.dataLevel) {
  case LEVEL0_ECHO:
    regData = CMD_LEVEL0_ECHO;
    break;
  case LEVEL0_DUAL_ECHO:
    regData= CMD_LEVLE0_DUAL_ECHO;
    break;
  case LEVEL1_ECHO:
    regData = CMD_LEVEL1_ECHO;
    break;
  case LEVEL1_DUAL_ECHO:
    regData = CMD_LEVEL1_DUAL_ECHO;
    break;
  case LEVEL2_ECHO:
    regData = CMD_LEVEL2_ECHO;
    break;
  case LEVEL2_DUAL_ECHO:
    regData = CMD_LEVEL2_DUAL_ECHO;
    break;
  case LEVEL3_ECHO:
    regData = CMD_LEVEL3_ECHO;
    break;
  case LEVEL3_DUAL_ECHO:
    regData = CMD_LEVEL3_DUAL_ECHO;
    break;
  default:
    break;
  }

  switch (program.cmdstat) {
  case eDevCmdWork:
    m_devapi->HW_WRREG(0, REG_DATA_LEVEL, regData);
    m_devapi->HW_WRREG(0, REG_DATA_LEVEL_OLD, regData);
    break;
  case eDevCmdAsk:
    break;
  default:
    break;
  }

  return 0;

}

void Rfans_Driver::configDeviceParams()
{
  int data_level = 3;
  bool dual_echo = config_.dual_echo;

  DEB_PROGRM_S params;
  params.cmdstat = eDevCmdWork;
  params.dataFormat = eFormatCalcData;

  // set start rps
  params.scnSpeed =  config_.scnSpeed;
  prog_Set(params);

  if (data_level== 0 && !dual_echo) {
    params.dataLevel = LEVEL0_ECHO;
  } else if (data_level == 0 && dual_echo) {
    params.dataLevel = LEVEL0_DUAL_ECHO;
  } else if (data_level == 1 && !dual_echo) {
    params.dataLevel = LEVEL1_ECHO;
  } else if (data_level == 1 && dual_echo) {
    params.dataLevel = LEVEL1_DUAL_ECHO;
  } else if(data_level == 2 && !dual_echo) {
    params.dataLevel = LEVEL2_ECHO;
  } else if (data_level == 2 && dual_echo) {
    params.dataLevel = LEVEL2_DUAL_ECHO;
  } else if (data_level == 3 && !dual_echo) {
    params.dataLevel = LEVEL3_ECHO;
  } else {
    params.dataLevel = LEVEL3_DUAL_ECHO;
  }

  // set data level
  datalevel_Set(params);
}

double Rfans_Driver::calcReplayPacketRate()
{
    double rate = 0.0f;
    std::string device = config_.device_name;
    double rate_angle = config_.angle_range/360.0;
    int data_level = 3;
    bool dual_echo = config_.dual_echo;
    //one second generate 640k points,and each packet have 32*12 points.
    if (device == "R-Fans-32") {
        if (((data_level == 0) || (data_level == 1)) && dual_echo) {
            rate = 6666.67;
        } else if (((data_level == 0)|| (data_level == 1)) && (!dual_echo)) {
            rate = 3333.33;
        } else if (data_level == 2 && dual_echo) {
            rate = 4000;
        } else if (data_level == 2 &&(!dual_echo)) {
            rate = 2000;
        } else if (data_level == 3 && dual_echo) {
            rate = 3333.33;
        } else if (data_level == 3 &&(!dual_echo)) {
            rate = 1666.67;
        } else if (data_level == 4 && dual_echo) {//FIXME: should check GM & BK format
            rate = 3333.33;
        }
        else{
            rate = 1666.67;
        }
    } else if (device == "R-Fans-16") {
        if (((data_level == 0)|| (data_level == 1)) && dual_echo) {
            rate = 3333.33;
        } else if (((data_level == 0)|| (data_level == 1)) && (!dual_echo)) {
            rate = 1666.67;
        } else if (data_level == 2 && dual_echo) {
            rate = 2000;
        } else if (data_level == 2 && (!dual_echo)) {
            rate = 1000;
        } else if (data_level == 3 && dual_echo) {
            rate = 1666.67;
        } else if (data_level == 3 && (!dual_echo)) {
            rate = 833.3;
        } else if (data_level == 4) {
            rate = 833.3;
        }
    }else if(device == "R-Fans-V6K"){
        if (data_level == 2 && dual_echo) {
            rate = 2133.33;
        } else if (data_level == 2 && (!dual_echo)) {
            rate = 1066.67;
        } else if (data_level == 3 && dual_echo) {
            rate = 1562.5;
        } else if (data_level == 3 && (!dual_echo)) {
            rate = 781.25;
        }
    }
    else if (device == "C-Fans-128" || device == "C-Fans-128-V2") {
        rate = 1666.67;
        // TODO:
    } else if (device == "C-Fans-32") {
        if(data_level == 2 && dual_echo){
            rate = 1000;
        } else if (data_level == 2 && (!dual_echo)) {
            rate = 500;
        } else if (data_level == 3 && dual_echo) {
            rate = 833.33;
        } else if (data_level == 3 && (!dual_echo)) {
            rate = 416.67;
        } else {
            // TODO:
            rate = 500;
        }
    } else {
        rate = 781.25;
    }
    rate *= rate_angle;
    return rate;
}



} //rfans_driver namespace
