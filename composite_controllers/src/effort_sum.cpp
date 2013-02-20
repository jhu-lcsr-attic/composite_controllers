
/**
   @class effort_controllers::JointPositionController
   @brief Joint Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "effort_controllers::JointPositionController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint position to achieve.

   Publishes:

   - @b state (controllers_msgs::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
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
      // Add services
      srv_add_subcontroller_ = cm_node_.advertiseService(
          "add_subcontroller",
          &EffortSum::add_subcontroller_srv, this);

      srv_del_subcontroller_ = cm_node_.advertiseService(
          "del_subcontroller",
          &EffortSum::del_subcontroller_srv, this);
    }

    ~EffortSum();

    bool init(hardware_interface::EffortJointInterface *effort_interface,
              ros::NodeHandle &nh)
    {
      // Get the list of joints that this effort sum will control
      XmlRpc::XmlRpcValue xml_joint_names;
      nh.getParam("joints", xml_joint_names);
      if(xml_joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("Joint name list parameter not found under: "<<nh.getNamespace()<<"/joints");
        return false;
      }

      for (unsigned int i = 0; i < xml_joint_names.size(); ++i) {
        ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        joint_names_.push_back(static_cast<std::string>(xml_joint_names[i]));
      }

      // Count the number of joints
      n_joints_ = joint_names_.size();

      // Acquire joint handles for thie actual hardware
      joint_handles_.resize(n_joints_);
      for(unsigned int j=0; j<n_joints; j++){
        // Note: calling getJointHandle claims this joint resource
        joint_handles_[j] = effort_interface->getJointHandle(joint_names_[j]);
      }

    }

    void starting(const ros::Time& time) {
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
      // Iterate through the subcontrollers
      for(std::map<std::string,SubController>::iterator subcontroller = subcontrollers_.begin();
          subcontroller != controller_managers_.end();
          ++subcontroller)
      {
        // Unpack
        EffortSumHWPtr hw = subcontroller->second->first;
        ControllerManagerPtr cm = subcontroller->second->second;

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
        joint_handles_[j].setCommand(effort_commands_[j]);
        // Reset the effort command
        effort_commands_[j] = 0.0;
      }
          
    }

    bool add_subcontroller(const std::string &name)
    {
      // Create a new EffortSumHW for the controller manager to control
      boost::shared_ptr<EffortSumHW> new_hw(new EffortSumHW());

      // Create a new controller manager
      ControllerManagerPtr new_manager(
          new controller_manager::ControllerManager(
              new_hw,
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
    std::vector<std::string> joint_names_;
    std::vector<double> joint_effort_command_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    typedef boost::shared_ptr<controller_manager::ControllerManager> ControllerManagerPtr;
    typedef boost::shared_ptr<EffortSumHW> EffortSumHWPtr;
    typedef std::pair<EffortSumHWPtr,ControllerManagerPtr> SubController;
    std::map<std::string,SubController> subcontrollers_;


    bool add_subcontroller_srv(
        controller_manager_msgs::LoadController::Request &req,
        controller_manager_msgs::LoadController::Response &resp)
    {
      resp.ok = true;
      return this->add_subcontroller(req.name);
    }

    bool del_subcontroller_srv(
        controller_manager_msgs::UnloadController::Request &req,
        controller_manager_msgs::UnloadController::Response &resp)
    {
      resp.ok = true;
      return this->del_subcontroller(req.name);
    }

  };

}

PLUGINLIB_DECLARE_CLASS(effort_controllers, JointPositionController, effort_controllers::JointPositionController, controller_interface::ControllerBase)
