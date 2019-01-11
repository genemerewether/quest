//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <algorithm>
#include <mav_msgs/AttitudeRateThrust.h>
#include <mav_msgs/FlatOutput.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Teleop {
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Publisher flatOutput_publisher_, attRateThrust_publisher_;
  ros::Time tLast_;
  double yaw_;

  struct Axis {
    Axis() : axis(0), factor(0.0), offset(0.0) {}

    int axis;
    double factor;
    double offset;
  };

  struct Button {
    Button() : button(0) {}

    int button;
  };

  struct {
    Axis x;
    Axis y;
    Axis z;
    Axis thrust;
    Axis yaw;
  } axes_;

  struct {
    Button slow;
    Button go;
    Button stop;
    Button interrupt;
  } buttons_;

  double slow_factor_;
  std::string base_link_frame_, base_stabilized_frame_, world_frame_;

public:
  Teleop() {
    ros::NodeHandle private_nh("~");

    private_nh.param<int>("x_axis", axes_.x.axis, 5);
    private_nh.param<int>("y_axis", axes_.y.axis, 4);
    private_nh.param<int>("z_axis", axes_.z.axis, 2);
    private_nh.param<int>("thrust_axis", axes_.thrust.axis, -3);
    private_nh.param<int>("yaw_axis", axes_.yaw.axis, 1);

    private_nh.param<double>("yaw_velocity_max", axes_.yaw.factor, 90.0);

    std::string control_mode;
    private_nh.param<std::string>("control_mode", control_mode, "attitude");

    ros::NodeHandle robot_nh;

    if (control_mode == "attitude") {
      private_nh.param<double>("pitch_max", axes_.x.factor, 30.0);
      private_nh.param<double>("roll_max", axes_.y.factor, 30.0);
      private_nh.param<double>("thrust_max", axes_.thrust.factor, 10.0);
      private_nh.param<double>("thrust_offset", axes_.thrust.offset, 10.0);

      joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>(
          "joy", 1, boost::bind(&Teleop::joyAttitudeCallback, this, _1));
      attRateThrust_publisher_ =
          robot_nh.advertise<mav_msgs::AttitudeRateThrust>(
              "attitude_rate_thrust_setpoint", 10);
    } else if (control_mode == "attrate") {
      private_nh.param<double>("pitchrate_max", axes_.x.factor, 90.0);
      private_nh.param<double>("rollrate_max", axes_.y.factor, 90.0);
      private_nh.param<double>("thrust_max", axes_.thrust.factor, 10.0);
      private_nh.param<double>("thrust_offset", axes_.thrust.offset, 10.0);

      joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>(
          "joy", 1, boost::bind(&Teleop::joyAttRateCallback, this, _1));
      attRateThrust_publisher_ =
          robot_nh.advertise<mav_msgs::AttitudeRateThrust>(
              "attitude_rate_thrust_setpoint", 10);
    } else if (control_mode == "velocity") {
      private_nh.param<double>("x_velocity_max", axes_.x.factor, 2.0);
      private_nh.param<double>("y_velocity_max", axes_.y.factor, 2.0);
      private_nh.param<double>("z_velocity_max", axes_.z.factor, 2.0);

    } else if (control_mode == "position") {
      private_nh.param<double>("x_velocity_max", axes_.x.factor, 2.0);
      private_nh.param<double>("y_velocity_max", axes_.y.factor, 2.0);
      private_nh.param<double>("z_velocity_max", axes_.z.factor, 2.0);

    } else {
      ROS_ERROR_STREAM("Unsupported control mode: " << control_mode);
    }
  }

  ~Teleop() {}

  void joyAttitudeCallback(const sensor_msgs::JoyConstPtr &joy) {
    ros::Time now = ros::Time::now();
    mav_msgs::AttitudeRateThrust attRateThrust;
    double dt = 0.0;
    if (!this->tLast_.isZero()) {
      dt = std::max(0.0, std::min(1.0, (now - this->tLast_).toSec()));
    }
    this->tLast_ = now;

    attRateThrust.header.stamp = now;
    attRateThrust.header.frame_id = "teleop";

    double roll = -getAxis(joy, axes_.y) * M_PI / 180.0;
    double pitch = getAxis(joy, axes_.x) * M_PI / 180.0;
    double yawrate = getAxis(joy, axes_.yaw) * M_PI / 180.0;
    this->yaw_ += yawrate * dt;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, this->yaw_);
    attRateThrust.attitude = tf2::toMsg(q);
    attRateThrust.thrust.z = getAxis(joy, axes_.thrust);
    attRateThrust.angular_rates.x = 0.0;
    attRateThrust.angular_rates.y = 0.0;
    attRateThrust.angular_rates.z = yawrate;

    attRateThrust_publisher_.publish(attRateThrust);
  }
  
  void joyAttRateCallback(const sensor_msgs::JoyConstPtr &joy) {
    ros::Time now = ros::Time::now();
    mav_msgs::AttitudeRateThrust attRateThrust;

    attRateThrust.header.stamp = now;
    attRateThrust.header.frame_id = "teleop";

    double roll = -getAxis(joy, axes_.y) * M_PI / 180.0;
    double pitch = getAxis(joy, axes_.x) * M_PI / 180.0;
    double yawrate = getAxis(joy, axes_.yaw) * M_PI / 180.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    attRateThrust.attitude = tf2::toMsg(q);
    attRateThrust.thrust.z = getAxis(joy, axes_.thrust);
    attRateThrust.angular_rates.x = roll;
    attRateThrust.angular_rates.y = pitch;
    attRateThrust.angular_rates.z = yawrate;

    attRateThrust_publisher_.publish(attRateThrust);
  }

  double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis) {
    if (axis.axis == 0 || std::abs(axis.axis) > joy->axes.size()) {
      ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has "
                               << joy->axes.size() << " axes");
      return 0;
    }

    double output = std::abs(axis.axis) / axis.axis *
                        joy->axes[std::abs(axis.axis) - 1] * axis.factor +
                    axis.offset;

    return output;
  }

  bool getButton(const sensor_msgs::JoyConstPtr &joy, const Button &button) {
    if (button.button <= 0 || button.button > joy->buttons.size()) {
      ROS_ERROR_STREAM("Button " << button.button << " out of range, joy has "
                                 << joy->buttons.size() << " buttons");
      return false;
    }

    return joy->buttons[button.button - 1] > 0;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop");

  Teleop teleop;
  ros::spin();

  return 0;
}
