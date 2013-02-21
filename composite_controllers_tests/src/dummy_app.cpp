///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

class TestHW : public hardware_interface::RobotHW
{
public:
  TestHW() :
    n_dof_(2),
    joint_name_(n_dof_),
    joint_position_(n_dof_),
    joint_velocity_(n_dof_),
    joint_effort_(n_dof_),
    joint_effort_command_(n_dof_)
  {

    joint_name_[0] = "joint1";
    joint_name_[1] = "joint2";

    for(unsigned int j=0; j < n_dof_; j++) {
      joint_position_[j] = 1.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 0.1;
      joint_effort_command_[j] = 0.0;

      js_interface_.registerJoint(joint_name_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]);
      ej_interface_.registerJoint(js_interface_.getJointStateHandle(joint_name_[j]), &joint_effort_command_[j]);
    }

    registerInterface(&js_interface_);
    registerInterface(&ej_interface_);
  }

  void read() {
  }

  void write() {
    ROS_INFO_STREAM("Effort command: ["<<joint_effort_command_[0]<<", "<<joint_effort_command_[1]<<"]");
  }

private:
  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;

  std::vector<std::string> joint_name_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DummyApp");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  TestHW hw;

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw, nh);

  ros::Duration period(1.0);
  while (ros::ok())
  {
    hw.read();
    cm.update(ros::Time::now(), period);
    hw.write();
    period.sleep();
  }
}

