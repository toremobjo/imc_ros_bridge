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

#include <imc_tcp_link/backseat_handle.hpp>
#include <IMC/Spec/Announce.hpp>
#include <IMC/Spec/Heartbeat.hpp>
#include <IMC/Spec/EntityInfo.hpp>

#include <functional>
#include <ros/ros.h>

void try_callback(const IMC::Message* imc_msg)
{
		ROS_INFO("Got callback!");
		std::cout << "Got callback with id: " << imc_msg->getId() << std::endl;
}

BackseatHandle::BackseatHandle(const std::string& bridge_tcp_addr,
					                     const std::string& bridge_tcp_port,
                               const std::string& sys_name)
    : bridge_tcp_addr_(bridge_tcp_addr), bridge_tcp_port_(bridge_tcp_port),
      sys_name_(sys_name)
{
    lat = 0.0;
    tcp_client_ = new ros_imc_broker::TcpLink(boost::bind(&BackseatHandle::tcp_callback, this, _1));
    tcp_client_->setServer(bridge_tcp_addr_, bridge_tcp_port_);
    tcp_client_thread_ = new boost::thread(boost::ref(*tcp_client_));
}

BackseatHandle::~BackseatHandle()
{
    if (tcp_client_thread_ == NULL)
      return;
    tcp_client_thread_->interrupt();
    tcp_client_thread_->join();
    delete tcp_client_thread_;
    tcp_client_thread_ = NULL;
    delete tcp_client_;
    tcp_client_ = NULL;
}

void BackseatHandle::tcp_subscribe(uint16_t uid, std::function<void(const IMC::Message*)> callback)
{
    callbacks[uid] = callback;
}

void BackseatHandle::tcp_callback(const IMC::Message* msg)
{
    uint16_t uid = msg->getId();
    if (callbacks.count(uid) > 0) {
	// 150 is a heartbeat and we dont really care about it. just debug it.
		if(uid == 150){
			ROS_DEBUG("Got callback with id: %u", uid);
		}else{
			ROS_INFO("Got callback with id: %u", uid);
		}
		callbacks.at(uid)(msg);
    }
    else {
        ROS_INFO("Got tcp message with no configure callback, msgid: %u!", uid);
    }
}

void BackseatHandle::announce()
{
    //std::string announce_addr = "224.0.75.69";
    //std::string announce_addr = "192.168.1.160";
/*IMC::Announce msg;
    msg.sys_name = sys_name;
    // 0=CCU, 1=HUMANSENSOR, 2 = UUV, 3 = ASV, 4=UAV, 5=UGV, 6=STATICSENSOR
    msg.sys_type = 2; // UUV = Unmanned underwater veh.
    msg.owner = 0;
    // dont put location info here, this is only updated once
    // use EstimatedState for continous updates of location.
    //lat +=0.01;
    //msg.lat = lat;
    //msg.lon = 0.7;
    //msg.height = -1.;
    //msg.services = "imc+info://0.0.0.0/version/5.4.11/;imc+udp://127.0.0.1:6002/;";
    msg.services = "imc+udp://" + bridge_tcp_addr + ":" + bridge_tcp_port + "/;";
    udp_link.publish_multicast(msg, neptus_addr);

    //TEST Publish EntityInfo
    IMC::EntityInfo info_msg;
    //info_msg.id = udp_link.imc_src; //What is this used for?
    info_msg.label = sys_name;
    udp_link.publish(info_msg, neptus_addr);*/
}

void BackseatHandle::publish_heartbeat()
{
    IMC::Heartbeat msg;
    ROS_WARN("yeeeeet!");
    //udp_link.publish(msg, neptus_addr);
}
