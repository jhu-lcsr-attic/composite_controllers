
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <pluginlib/class_list_macros.h>

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
    class EffortSumHW : public hardware_interface::RobotHW {
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

    EffortSum()
    {
    }

    ~EffortSum();

    bool init(hardware_interface::EffortJointInterface *effort_interface,
              ros::NodeHandle &nh)
    {
      // Grab the handle
      nh_ = nh;

      // Get the list of joints that this effort sum will control
      XmlRpc::XmlRpcValue xml_joint_names;
      nh.getParam("joints", xml_joint_names);
      if(xml_joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("Joint name list parameter not found under: "<<nh.getNamespace()<<"/joints");
        return false;
      }

      joint_names_.resize(xml_joint_names.size());
      for (int j = 0; j < xml_joint_names.size(); ++j) {
        ROS_ASSERT(xml_joint_names[j].getType() == XmlRpc::XmlRpcValue::TypeString);
        joint_names_[j] = static_cast<std::string>(xml_joint_names[j]);
      }

      // Count the number of joints
      n_joints_ = joint_names_.size();

      // Acquire joint handles for thie actual hardware
      joint_handles_.resize(n_joints_);
      for(unsigned int j=0; j<n_joints_; j++){
        // Note: calling getJointHandle claims this joint resource
        joint_handles_[j] = effort_interface->getJointHandle(joint_names_[j]);
      }

      // Add services
      add_subcontroller_srv_ = nh.advertiseService(
          "add_subcontroller",
          &EffortSum::add_subcontroller_srv_cb, this);

      del_subcontroller_srv_ = nh.advertiseService(
          "del_subcontroller",
          &EffortSum::del_subcontroller_srv_cb, this);

      return true;
    }

    void starting(const ros::Time& time) {
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      // Iterate through the subcontrollers
      for(std::map<std::string,SubController>::iterator subcontroller = subcontrollers_.begin();
          subcontroller != subcontrollers_.end();
          ++subcontroller)
      {
        // Unpack
        EffortSumHWPtr hw = subcontroller->second.first;
        ControllerManagerPtr cm = subcontroller->second.second;

        // Read the state from the lower-level controller
        hw->read_state(joint_handles_);
        // Update the higher-level controllers
        cm->update(time, period);
        // Add the efforts from this manager 
        hw->add_effort(joint_effort_command_);
      }
      
      // Iterate through the joints
      for(unsigned int j=0; j < n_joints_; j++) {
        // Set the effort for the lower-level controller
        joint_handles_[j].setCommand(joint_effort_command_[j]);
        // Reset the effort command
        joint_effort_command_[j] = 0.0;
      }
          
    }

    bool add_subcontroller(const std::string &name)
    {
      // Create a new EffortSumHW for the controller manager to control
      boost::shared_ptr<EffortSumHW> new_hw(new EffortSumHW(joint_names_));

      // Create a new controller manager
      ControllerManagerPtr new_manager(
          new controller_manager::ControllerManager(
              new_hw.get(),
              ros::NodeHandle(nh_,name)));

      // Store them
      subcontrollers_[name] = std::make_pair(new_hw,new_manager);

      return true;
    }

    bool del_subcontroller(const std::string &name)
    {
      // Check if this subcontroller exists
      if(subcontrollers_.count(name)) {
        ROS_INFO_STREAM("Deleteing subcontroller: \""<<name<<"\""); 
        subcontrollers_.erase(name);
      } else {
        ROS_WARN_STREAM("Trying to delete non-existent subcontroller: \""<<name<<"\"");
        return false;
      }

      return true;
    }

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
        composite_controllers_msgs::AddSubcontroller::Response &resp)
    {
      resp.ok = this->add_subcontroller(req.name);
      return resp.ok;
    }

    bool del_subcontroller_srv_cb(
        composite_controllers_msgs::DelSubcontroller::Request &req,
        composite_controllers_msgs::DelSubcontroller::Response &resp)
    {
      resp.ok = this->del_subcontroller(req.name);
      return resp.ok;
    }

  };

}

PLUGINLIB_DECLARE_CLASS(composite_controllers, EffortSum, composite_controllers::EffortSum, controller_interface::ControllerBase)
