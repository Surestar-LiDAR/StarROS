/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <ros/ros.h>
#include "rfans_driver.h"
#include <dlfcn.h>
ros::Publisher  sdk_output;
double min_range;
double max_range;
double min_angle;
double max_angle;
int ringID;
bool use_laserSelection_;
bool distortion_flag;
float Angle_resolution;
bool Device;
void callback(rfans_driver::FilterParamsConfig &config, uint32_t level){
  min_range = config.min_range;
  max_range = config.max_range;
  if(Device)
  {
    config.rfans_angleSelection=true;
    min_angle = config.min_angle;
    max_angle = config.max_angle;
    config.cfans_angleSelection =false;
  }
  else
  {
    config.cfans_angleSelection =true;
    min_angle = config.cfans_min_angle;
    max_angle = config.cfans_max_angle;
    config.rfans_angleSelection = false;
  }

  use_laserSelection_ = config.use_laserSelection;
  ringID = config.laserID;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rfans_driver");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
//  ROS_INFO("pacp:");
//  auto dll = dlopen("libpcap.so", RTLD_LAZY);
//  ROS_INFO("dll");
//  if(dll)
//  {
//    ROS_INFO("success");
//  }
//  dlsym(dll, "pcap_open_offline");
//  dlsym(dll, "pcap_fopen_offline");
//  dlsym(dll, "pcap_close");
//  dlsym(dll, "pcap_next_ex");
//  dlsym(dll, "pcap_loop");
//  dlsym(dll, "pcap_setnonblock");
//  dlsym(dll, "pcap_stats");
//   ROS_INFO("success1");
//  // /dlclose(so);

//  if (pcap_fopen_offline == nullptr || pcap_close == nullptr || pcap_loop == nullptr) {
//    return false;
//  }
//  Pcap* stream = new Pcap("/home/bkth/qt_ws/pcap/32G.pcap");
//  ROS_INFO("stream:");
  rfans_driver::Rfans_Driver* driver = new rfans_driver::Rfans_Driver(node, nh);
  sdk_output = node.advertise<sensor_msgs::PointCloud2>("sdk_could", 10000);


  dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig> server;
  dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig>::CallbackType f;
  f = boost::bind(&callback,_1,_2);
  server.setCallback(f);


  while (ros::ok()&&driver->spinOnce())
  {
  }


  return 0;
}
