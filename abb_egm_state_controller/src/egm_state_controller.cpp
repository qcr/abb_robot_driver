/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <pluginlib/class_list_macros.hpp>

#include <abb_robot_cpp_utilities/mapping.h>
#include <abb_robot_cpp_utilities/parameters.h>
#include <abb_robot_cpp_utilities/verification.h>
// Added for debugging
#include <google/protobuf/text_format.h>

#include "abb_egm_state_controller/egm_state_controller.h"

namespace
{
/**
 * \brief Name for ROS logging in the 'init' context.
 */
constexpr char ROS_LOG_INIT[]{"init"};
}

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Class definitions: EGMStateController
 */

/***********************************************************
 * Primary methods
 */

bool EGMStateController::init(EGMStateInterface* p_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  //--------------------------------------------------------
  // Verify the hardware interface
  //--------------------------------------------------------
  if(!p_hw)
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_INIT, "Hardware interface has not been not allocated");
    return false;
  }

  // Verify that the interface have one or more resources.
  if(p_hw->getNames().size() >= 1)
  {
    try
    {
      // Get the handles (throws an exception on failure).
      for(const auto& name : p_hw->getNames())
      {
        egm_state_handles_.push_back(p_hw->getHandle(name));
      }
    }
    catch(...)
    {
      ROS_FATAL_STREAM_NAMED(ROS_LOG_INIT, "Failed to get resource handles from hardware interface");
      return false;
    }
  }
  else
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_INIT, "Hardware interface has less than 1 resources");
    return false;
  }

  //--------------------------------------------------------
  // Get parameter from the parameter server
  //--------------------------------------------------------
  try
  {
    double publish_rate{DEFAULT_PUBLISH_RATE};
    utilities::getParameter(controller_nh, "publish_rate", publish_rate, DEFAULT_PUBLISH_RATE);
    utilities::verifyRate(publish_rate);
    publish_period_ = ros::Duration(1.0/publish_rate);
  }
  catch(...)
  {
    return false;
  }

  //--------------------------------------------------------
  // Setup publisher(s)
  //--------------------------------------------------------
  p_egm_state_publisher_.reset(new EGMStatePublisher(root_nh, "egm_states", 1));
  // [09/03/22] Added new publisher (realtime) object for force data
  p_egm_force_publisher_.reset(new EGMForcePublisher(root_nh, "egm_forces", 1));

  // Allocate ROS messages.
  for(const auto& handle : egm_state_handles_)
  {
    auto p_data{handle.getEGMChannelDataPtr()};

    abb_egm_msgs::EGMChannelState channel{};
    // [09/03/22] Added for force output
    abb_custom_msgs::EGMChannelForces channel_forces{};

    // General information.
    // [09/03/22] Added for force output
    channel.name = handle.getName();
    channel_forces.name = handle.getName();
    channel.active = p_data->is_active;
    channel_forces.active = p_data->is_active;

    // Status information.
    channel.egm_convergence_met = p_data->input.status().egm_convergence_met();
    channel.egm_client_state = utilities::map(p_data->input.status().egm_state());
    channel.motor_state = utilities::map(p_data->input.status().motor_state());
    channel.rapid_execution_state = utilities::map(p_data->input.status().rapid_execution_state());
    channel.utilization_rate = p_data->input.status().utilization_rate();

    p_egm_state_publisher_->msg_.egm_channels.push_back(channel);

    // Force information
  #if 1
    //DEBUGGING SECTION
    abb::egm::wrapper::MeasuredForce forces;
    forces.CopyFrom(p_data->input.measuredforce());
    google::protobuf::RepeatedField<double> measured_forces = forces.force();

    std::string debug_string;
    google::protobuf::TextFormat::PrintToString(forces, &debug_string);
    std::cout << "Measured Forces:\n" << debug_string << std::endl;
  #endif
    if(measured_forces.size() == 6)
    {
      channel_forces.linear_x = measured_forces[0];
      channel_forces.linear_y = measured_forces[1];
      channel_forces.linear_z = measured_forces[2];
      channel_forces.torque_x = measured_forces[3];
      channel_forces.torque_y = measured_forces[4];
      channel_forces.torque_z = measured_forces[5];
    }
    else
    {
      channel_forces.linear_x = -10.0;
      channel_forces.linear_y = -10.0;
      channel_forces.linear_z = -10.0;
      channel_forces.torque_x = -10.0;
      channel_forces.torque_y = -10.0;
      channel_forces.torque_z = -10.0;  
    }

    p_egm_force_publisher_->msg_.egm_channel_forces.push_back(channel_forces);
  }

  return true;
}

void EGMStateController::starting(const ros::Time& time) { last_publish_time_ = time; }

void EGMStateController::update(const ros::Time& time, const ros::Duration& period)
{
  (void) period;

  if(last_publish_time_ + publish_period_ < time)
  {
    last_publish_time_ += publish_period_;

    //------------------------------------------------------
    // Try to publish EGM state
    //------------------------------------------------------
    if(p_egm_state_publisher_->trylock())
    {
      p_egm_state_publisher_->msg_.header.stamp = time;

      for(size_t i = 0; i < egm_state_handles_.size() && i < p_egm_state_publisher_->msg_.egm_channels.size(); ++i)
      {
        auto p_data{egm_state_handles_[i].getEGMChannelDataPtr()};

        // Status information.
        auto& channel{p_egm_state_publisher_->msg_.egm_channels[i]};
        channel.active = p_data->is_active;

        // Status information.
        channel.egm_convergence_met = p_data->input.status().egm_convergence_met();
        channel.egm_client_state = utilities::map(p_data->input.status().egm_state());
        channel.motor_state = utilities::map(p_data->input.status().motor_state());
        channel.rapid_execution_state = utilities::map(p_data->input.status().rapid_execution_state());
        channel.utilization_rate = p_data->input.status().utilization_rate();
      }

      p_egm_state_publisher_->unlockAndPublish();
    }

    //-----------------------------------------------------
    // [09/03/22] Added for Force Information - Try to publish
    //-----------------------------------------------------
    if(p_egm_force_publisher_->trylock())
    {
      p_egm_force_publisher_->msg_.header.stamp = time;
      for(size_t i = 0; i < egm_state_handles_.size() && i < p_egm_force_publisher_->msg_.egm_channel_forces.size(); ++i)
      {
        auto p_data{egm_state_handles_[i].getEGMChannelDataPtr()};

        // Status information.
        auto& channel{p_egm_force_publisher_->msg_.egm_channel_forces[i]};
        channel.active = p_data->is_active;

        // Force information
#if 1
        //DEBUGGING      
        abb::egm::wrapper::MeasuredForce forces;
        forces.CopyFrom(p_data->input.measuredforce());
        google::protobuf::RepeatedField<double> measured_forces = forces.force();

        std::string debug_string;
        google::protobuf::TextFormat::PrintToString(forces, &debug_string);
        std::cout << "Measured Forces:\n" << debug_string << std::endl;
#endif
        if(measured_forces.size() == 6)
        {
          channel.linear_x = measured_forces[0];
          channel.linear_y = measured_forces[1];
          channel.linear_z = measured_forces[2];
          channel.torque_x = measured_forces[3];
          channel.torque_y = measured_forces[4];
          channel.torque_z = measured_forces[5];
        }
        else
        {
          channel.linear_x = -10.0;
          channel.linear_y = -10.0;
          channel.linear_z = -10.0;
          channel.torque_x = -10.0;
          channel.torque_y = -10.0;
          channel.torque_z = -10.0;  
        }
      }

      p_egm_force_publisher_->unlockAndPublish();
    }
  }
}

void EGMStateController::stopping(const ros::Time& time) { (void) time; }

}
}

PLUGINLIB_EXPORT_CLASS(abb::robot::EGMStateController, controller_interface::ControllerBase)
