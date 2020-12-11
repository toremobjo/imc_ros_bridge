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


#ifndef ROS_IMC_BACKSEAT_DRIVER_HPP_INCLUDED_
#define ROS_IMC_BACKSEAT_DRIVER_HPP_INCLUDED_

#include <ros/ros.h>
#include <iostream>

#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>
#include <imc_ros_bridge/ros_to_imc/GpsFix.h>
#include <imc_ros_bridge/ros_to_imc/Goto.h>
#include <imc_ros_bridge/ros_to_imc/GpsNavData.h>
#include <imc_ros_bridge/ros_to_imc/EstimatedState.h>
#include <imc_ros_bridge/ros_to_imc/PlanControlState.h>
#include <imc_ros_bridge/ros_to_imc/VehicleState.h>
#include <imc_ros_bridge/ros_to_imc/RemoteState.h>
#include <imc_ros_bridge/ros_to_imc/SonarData.h>
#include <imc_ros_bridge/ros_to_imc/DesiredHeading.h>
#include <imc_ros_bridge/ros_to_imc/DesiredHeadingRate.h>
#include <imc_ros_bridge/ros_to_imc/DesiredPitch.h>
#include <imc_ros_bridge/ros_to_imc/DesiredRoll.h>
#include <imc_ros_bridge/ros_to_imc/DesiredSpeed.h>
#include <imc_ros_bridge/ros_to_imc/DesiredZ.h>
#include <imc_ros_bridge/ros_to_imc/PlanDB.h>

#include <imc_ros_bridge/imc_to_ros/Goto.h>
#include <imc_ros_bridge/imc_to_ros/Heartbeat.h>
#include <imc_ros_bridge/imc_to_ros/Abort.h>
#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <imc_ros_bridge/imc_to_ros/PlanControl.h>



#endif
