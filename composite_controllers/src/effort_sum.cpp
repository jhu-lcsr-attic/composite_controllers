
#include <composite_controllers/effort_sum.h>
#include <pluginlib/class_list_macros.h>


namespace composite_controllers {
  EffortSum::EffortSum()
  {
  }

  EffortSum::~EffortSum()
  {
  }

  bool EffortSum::init(hardware_interface::EffortJointInterface *effort_interface,
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

    joint_effort_command_.assign(xml_joint_names.size(),0.0);

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

  void EffortSum::starting(const ros::Time& time)
  {
  }

  void EffortSum::update(const ros::Time& time, const ros::Duration& period)
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

  bool EffortSum::add_subcontroller(const std::string &name)
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

  bool EffortSum::del_subcontroller(const std::string &name)
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

  bool EffortSum::add_subcontroller_srv_cb(
      composite_controllers_msgs::AddSubcontroller::Request &req,
      composite_controllers_msgs::AddSubcontroller::Response &resp)
  {
    resp.ok = this->add_subcontroller(req.name);
    return resp.ok;
  }

  bool EffortSum::del_subcontroller_srv_cb(
      composite_controllers_msgs::DelSubcontroller::Request &req,
      composite_controllers_msgs::DelSubcontroller::Response &resp)
  {
    resp.ok = this->del_subcontroller(req.name);
    return resp.ok;
  }
}

PLUGINLIB_DECLARE_CLASS(composite_controllers, EffortSum, composite_controllers::EffortSum, controller_interface::ControllerBase)
