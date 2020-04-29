/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include <stdio.h>
#include <time.h>

#include "rfans_driver/RfansCommand.h"
#include "ioapi.h"

int main ( int argc , char ** argv ) 
{
  if(argc != 3 ) {
    std::cout<< "Usage: rfans_subscriber [cmd] [speed]\n"
             << "     Parameter:\n"
             << "       cmd,   0: stop RFans; 1:start RFans\n"
             << "     speed,   5: 5hz; 10: 10hz; 20: 20hz\n";

    return 0;
  }

  ros::init ( argc , argv , "rfans_subscribe") ;
  ros::NodeHandle nh ;


  ros::ServiceClient client = nh.serviceClient<rfans_driver::RfansCommand>("rfans_driver/rfans_control");
  rfans_driver::RfansCommand srv;
  srv.request.cmd = atoll(argv[1]);
  srv.request.speed = atoll(argv[2]);
  if (client.call(srv)) {
    ROS_INFO("status: %ld", (long int)srv.response.status);
  } else {
    ROS_ERROR("Failed to call service RfansCommand,Please");
  }
  ros::spin () ;
}
