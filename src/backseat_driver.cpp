/* Copyright 2019 The SMaRC project (https://smarc.se/)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <iostream>

#include <imc_ros_bridge/backseat_driver.hpp>
#include <imc_tcp_link/backseat_handle.hpp>
#include <imc_ros_bridge/imc_ros_bridge_server.h>


class BackseatDriver
{
public:
  BackseatDriver(ros::NodeHandle& nh, const std::string& system_name,const std::string& system_ip,const std::string& system_port):
  nh_(nh),
  system_name_(system_name),
  system_ip_(system_ip),
  system_port_(system_port)
    {
      int b;
      BackseatHandle bh(system_ip_,system_port_,system_name_);
    }

  virtual
  ~BackseatDriver(void){}


private:
  //! ROS node handle.
  ros::NodeHandle& nh_;
  //! Canonical name of the controlled system.
  std::string system_name_;
  //! IP address of the system
  std::string system_ip_;
  //! Port of Backseat driver
  std::string system_port_;
};



using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "backseat_driver");
  ros::NodeHandle nh_;

  string system_ip;
  string system_port;
  string system_name;

  ros::param::param<std::string>("~system_ip", system_ip, "127.0.0.1");
  ros::param::param<std::string>("~system_port", system_port, "32603");
  ros::param::param<std::string>("~system_name", system_name, "lauv-simulator-1");


  BackseatDriver bd(nh_, system_name, system_ip, system_port);


  ros::spin();
  return 0;
}
