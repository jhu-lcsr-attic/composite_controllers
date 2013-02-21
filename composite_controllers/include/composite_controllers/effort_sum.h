#ifndef __COMPOSITE_CONTROLLERS_EFFORT_SUM_H
#define __COMPOSITE_CONTROLLERS_EFFORT_SUM_H

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <controller_interface/controller.h>
#include <controller_manager/controller_manager.h>
#include <composite_controllers_msgs/AddSubcontroller.h>
#include <composite_controllers_msgs/DelSubcontroller.h>
#include <hardware_interface/joint_command_interface.h>

namespace composite_controllers 
{
  class EffortSum : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
  public:
    class EffortSumHW : public hardware_interface::RobotHW 
    {
      public: 
        EffortSumHW(const std::vector<std::string> joint_names) :
          joint_position_(joint_names.size()),
          joint_velocity_(joint_names.size()),
          joint_effort_(joint_names.size()),
          joint_effort_command_(joint_names.size())
        {
          for(unsigned int j=0; j<joint_names.size(); j++) {
            // Register joint with state interface
            state_interface_.registerJoint(
                joint_names[j],
                &joint_position_[j],
                &joint_velocity_[j], 
                &joint_effort_[j]);

            // Register joint with effort interface
            effort_interface_.registerJoint(
                state_interface_.getJointStateHandle(joint_names[j]),
                &joint_effort_command_[j]);
          }

          // Register the interfaces
          registerInterface(&state_interface_);
          registerInterface(&effort_interface_);
        }

        void read_state(const std::vector<hardware_interface::JointHandle> &joint_handles) {
          for(unsigned int j=0; j<joint_handles.size(); j++) {
            joint_position_[j] = joint_handles[j].getPosition();
            joint_velocity_[j] = joint_handles[j].getVelocity();
            joint_effort_[j] = joint_handles[j].getEffort();
          }
        }

        void add_effort(std::vector<double> &joint_effort_command) {
          for(unsigned int j=0; j<joint_effort_command.size(); j++) {
            joint_effort_command[j] += joint_effort_command_[j];
          }
        }
      private:
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_effort_command_;
        hardware_interface::JointStateInterface state_interface_;
        hardware_interface::EffortJointInterface effort_interface_;
    };

    EffortSum();

    ~EffortSum();

    bool init(hardware_interface::EffortJointInterface *effort_interface,
              ros::NodeHandle &nh);

    void starting(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    bool add_subcontroller(const std::string &name);

    bool del_subcontroller(const std::string &name);

  private:
    ros::NodeHandle nh_;
    unsigned int n_joints_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_effort_command_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    typedef boost::shared_ptr<controller_manager::ControllerManager> ControllerManagerPtr;
    typedef boost::shared_ptr<EffortSumHW> EffortSumHWPtr;
    typedef std::pair<EffortSumHWPtr,ControllerManagerPtr> SubController;
    std::map<std::string,SubController> subcontrollers_;

    ros::ServiceServer add_subcontroller_srv_, del_subcontroller_srv_;

    bool add_subcontroller_srv_cb(
        composite_controllers_msgs::AddSubcontroller::Request &req,
        composite_controllers_msgs::AddSubcontroller::Response &resp);

    bool del_subcontroller_srv_cb(
        composite_controllers_msgs::DelSubcontroller::Request &req,
        composite_controllers_msgs::DelSubcontroller::Response &resp);

  };
}

#endif // ifndef __COMPOSITE_CONTROLLERS_EFFORT_SUM_H
