/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Softbank Corp. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/** NOTE *************************************************************
 * This program had been developed by Michael T. Boulet at MIT under
 * the BSD 3-clause License until Dec. 2016. Since Nov. 2019, Softbank
 * Corp. takes over development as new packages.
 ********************************************************************/

#include "vesc_driver/vesc_driver.h"

namespace vesc_driver {
    VescDriver::VescDriver(ros::NodeHandle &nh, ros::NodeHandle &private_nh, std::string inter_name, int can_dev_id)
              :vesc_(),duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0), current_limit_(private_nh, "current"),
              brake_limit_(private_nh, "brake"), speed_limit_(private_nh, "speed"),
              position_limit_(private_nh, "position") 
    {

      vesc_.start(inter_name, can_dev_id);


      // create vesc state (telemetry) publisher
      state_pub_ = private_nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 10);

      // subscribe to motor and servo command topics
      duty_cycle_sub_ = private_nh.subscribe("duty_cycle", 10, &VescDriver::dutyCycleCallback, this);
      current_sub_ = private_nh.subscribe("current", 10, &VescDriver::currentCallback, this);
      brake_sub_ = private_nh.subscribe("brake", 10, &VescDriver::brakeCallback, this);
      speed_sub_ = private_nh.subscribe("speed", 10, &VescDriver::speedCallback, this);
      position_sub_ = private_nh.subscribe("position", 10, &VescDriver::positionCallback, this);
        
    }

    void VescDriver::vescErrorCallback(const std::string &error) {
        ROS_ERROR("%s", error.c_str());
    }

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */

    void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr &duty_cycle) 
    {
      vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
    }

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
    void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr &current) 
    {
        vesc_.setCurrent(current_limit_.clip(current->data));
    }
//
// /**
//  * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
//  *              However, note that the VESC may impose a more restrictive bounds on the range
//  *              depending on its configuration.
//  */
    void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr &brake) 
    {
      vesc_.setBrake(brake_limit_.clip(brake->data));
    }
//
// /**
//  * @param speed Commanded VESC speed in rad/s. Although any value is accepted by this driver, the VESC may impose a
//  *              more restrictive bounds on the range depending on its configuration.
//  */
    void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr &speed) 
    {
      vesc_.setSpeed(speed_limit_.clip(speed->data));
    }
//
// /**
//  * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
//  *                 Note that the VESC must be in encoder mode for this command to have an effect.
//  */
    void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr &position) 
    {
      // ROS uses radians but VESC seems to use degrees. Convert to degrees.
      // double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
      double position_deg = position_limit_.clip(position->data);
      vesc_.setPosition(position_deg);
      // printf("Pos value: %f\n", position_deg);
      // printf("Pos ang: %f\n", position->data);
    }
//
    void VescDriver::waitForStateAndPublish() {
        vesc_.wait_for_status(&vesc_status);

        vesc_msgs::VescStateStamped::Ptr state_msg(new vesc_msgs::VescStateStamped);
        state_msg->header.stamp = ros::Time::now();
        state_msg->state.connection_state = vesc_status.connection_state;
        state_msg->state.fw_major = vesc_status.fw_version_major;
        state_msg->state.fw_minor = vesc_status.fw_version_minor;
        state_msg->state.voltage_input = vesc_status.voltage_input;
        state_msg->state.temperature_pcb = vesc_status.temperature_pcb;
        state_msg->state.current_motor = vesc_status.current_motor;
        state_msg->state.current_input = vesc_status.current_input;
        state_msg->state.speed = vesc_status.speed_erpm;
        state_msg->state.duty_cycle = vesc_status.duty_cycle;
        state_msg->state.charge_drawn = vesc_status.charge_drawn;
        state_msg->state.charge_regen = vesc_status.charge_regen;
        state_msg->state.energy_drawn = vesc_status.energy_drawn;
        state_msg->state.energy_regen = vesc_status.energy_regen;
        state_msg->state.displacement = vesc_status.current_pid_position;
        state_msg->state.distance_traveled = vesc_status.distance_traveled;
        state_msg->state.adc_1 = vesc_status.ext_adc1;
        state_msg->state.adc_2 = vesc_status.ext_adc2;
        state_msg->state.adc_3 = vesc_status.ext_adc3;
        state_msg->state.fault_code = vesc_status.fault_code;

        state_pub_.publish(state_msg);
    }

    void VescDriver::stop() 
    {
      vesc_.stop();
    }


    VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle &nh, const std::string &str,
                                           const boost::optional<double> &min_lower,
                                           const boost::optional<double> &max_upper)
            : name(str) {
        // check if user's minimum value is outside of the range min_lower to max_upper
        double param_min;
        if (nh.getParam(name + "_min", param_min)) {
            if (min_lower && param_min < *min_lower) {
                lower = *min_lower;
                ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min << ") is less than the feasible minimum ("
                                             << *min_lower << ").");
            } else if (max_upper && param_min > *max_upper) {
                lower = *max_upper;
                ROS_WARN_STREAM(
                        "Parameter " << name << "_min (" << param_min << ") is greater than the feasible maximum ("
                                     << *max_upper << ").");
            } else {
                lower = param_min;
            }
        } else if (min_lower) {
            lower = *min_lower;
        }

        // check if the uers' maximum value is outside of the range min_lower to max_upper
        double param_max;
        if (nh.getParam(name + "_max", param_max)) {
            if (min_lower && param_max < *min_lower) {
                upper = *min_lower;
                ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max << ") is less than the feasible minimum ("
                                             << *min_lower << ").");
            } else if (max_upper && param_max > *max_upper) {
                upper = *max_upper;
                ROS_WARN_STREAM(
                        "Parameter " << name << "_max (" << param_max << ") is greater than the feasible maximum ("
                                     << *max_upper << ").");
            } else {
                upper = param_max;
            }
        } else if (max_upper) {
            upper = *max_upper;
        }

        // check for min > max
        if (upper && lower && *lower > *upper) {
            ROS_WARN_STREAM(
                    "Parameter " << name << "_max (" << *upper << ") is less than parameter " << name << "_min ("
                                 << *lower << ").");
            double temp(*lower);
            lower = *upper;
            upper = temp;
        }

        std::ostringstream oss;
        oss << "  " << name << " limit: ";
        if (lower)
            oss << *lower << " ";
        else
            oss << "(none) ";
        if (upper)
            oss << *upper;
        else
            oss << "(none)";
        ROS_DEBUG_STREAM(oss.str());
    }

    double VescDriver::CommandLimit::clip(double value) {
        if (lower && value < lower) {
            ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.", name.c_str(), value,
                              *lower);
            return *lower;
        }
        if (upper && value > upper) {
            ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.", name.c_str(), value,
                              *upper);
            return *upper;
        }
        return value;
    }

}  // namespace vesc_driver
