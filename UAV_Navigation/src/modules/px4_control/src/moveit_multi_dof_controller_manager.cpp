/**
 * @file moveit_multi_dof_controller_manager.cpp
 * @brief Controller manager for multi DOF joint
 */

#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include "follow_multi_dof_joint_trajectory_controller_handle.hpp"
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.hpp>

namespace dronedoc
{

class MoveItMultiDOFControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  /**
   * @brief Default constructor
   */
  MoveItMultiDOFControllerManager() : node_handle_("~")
  {
    // Error if controller_list param is not set
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM_NAMED("manager", "No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);

    // Error if controller_list is not an array
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Parameter controller_list should be specified as an array");
      return;
    }

    // Error if multiple controller is defined
    if(controller_list.size() > 1)
    {
      ROS_ERROR("This controller manager expects only one controller.");
      return;
    }

    // Error if controller not have required fields
    if (!controller_list[0].hasMember("name") || !controller_list[0].hasMember("joints"))
    {
      ROS_ERROR_STREAM_NAMED("manager", "Name and joints must be specifed for each controller");
      return;
    }

    try
    {
      std::string name = std::string(controller_list[0]["name"]);

      std::string action_ns;

      // Warn if controller has "ns" field
      if (controller_list[0].hasMember("ns"))
      {
        action_ns = std::string(controller_list[0]["ns"]);
        ROS_WARN_NAMED("manager", "Use of 'ns' is deprecated, use 'action_ns' instead.");
      } // Set action namespace
      else if (controller_list[0].hasMember("action_ns"))
        action_ns = std::string(controller_list[0]["action_ns"]);
      else // Warn if "action_ns" not specified
        ROS_WARN_NAMED("manager", "Please note that 'action_ns' no longer has a default value.");

      // Error if "joints" field is not array
      if (controller_list[0]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM_NAMED("manager", "The list of joints for controller " << name
                                                                               << " is not specified as an array");
        return;
      }

      // Error if controller not have "type" field
      if (!controller_list[0].hasMember("type"))
      {
        ROS_ERROR_STREAM_NAMED("manager", "No type specified for controller " << name);
        return;
      }

      std::string type = std::string(controller_list[0]["type"]);

      // Set controller handle if "type" is FollowMultiDOFJointTrajectory
      moveit_simple_controller_manager::ActionBasedControllerHandleBasePtr new_handle;
      if (type == "FollowMultiDOFJointTrajectory")
      {
        new_handle.reset(new FollowMultiDOFJointTrajectoryControllerHandle(name, action_ns));
        if (static_cast<FollowMultiDOFJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
        {
          ROS_INFO_STREAM_NAMED("manager", "Added FollowJointTrajectory controller for " << name);
          controller_ = new_handle;
        }
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("manager", "Unknown controller type: " << type.c_str());
        return;
      }

      /* add list of joints, used by controller manager and MoveIt! */
      for (int i = 0; i < controller_list[0]["joints"].size(); ++i)
        controller_->addJoint(std::string(controller_list[0]["joints"][i]));
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED("manager", "Caught unknown exception while parsing controller information");
    }
  }

  /**
   * @brief Destructor
   */
  ~MoveItMultiDOFControllerManager() override
  {
  }

  /**
   * @brief Returns pointer to controller handle
   * @param name
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(controller_);
  }

  /**
   * @brief Add FollowMultiDOFJointTrajectory to controller list
   * @param names
   *
   * This manager only deals FollowMultiDOFJointTrajectory controller
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    names.push_back("FollowMultiDOFJointTrajectory");
  }

  /**
   * @brief Get all controllers
   * @param names
   *
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal
   * with it anyways!
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  /**
   * @brief Get all controllers
   * @param names
   *
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /**
   * @brief Get the list of joints that a controller can control.
   * @param name Controller name
   * @param joints List of joints
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    if (name == "FollowMultiDOFJointTrajectory")
    {
      controller_->getJoints(joints);
    }
    else
    {
      ROS_WARN_NAMED("manager", "The joints for controller '%s' are not known. Perhaps the controller configuration is "
                                "not loaded on the param server?",
                     name.c_str());
      joints.clear();
    }
  }

  /**
   * @brief Get state of controller specified by name
   * @param name
   *
   * Controllers are all active and default.
   */
  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name) override
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /**
   * @brief Switch controllers
   * @param activate
   * @param deactivate
   *
   * Cannot switch our controllers
   */
  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) override
  {
    return false;
  }

protected:
  ros::NodeHandle node_handle_;
  moveit_simple_controller_manager::ActionBasedControllerHandleBasePtr controller_;
};

}  // end namespace dronedoc

PLUGINLIB_EXPORT_CLASS(dronedoc::MoveItMultiDOFControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
