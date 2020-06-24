/**
 * @file follow_multi_dof_joint_trajectory_controller_handle.cpp
 * @brief Action client for ExecuteTrajectory action
 */

#ifndef MOVEIT_PLUGINS_FOLLOW_MULTI_DOF_TRAJECTORY_CONTROLLER_HANDLE
#define MOVEIT_PLUGINS_FOLLOW_MULTI_DOF_TRAJECTORY_CONTROLLER_HANDLE

#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

namespace dronedoc
{
/**
 * @brief Controller for multi DOF joint trajectory
 *
 * This is generally used for arms, but could also be used for multi-dof hands,
 * or anything using a control_mgs/FollowJointTrajectoryAction.
 */
class FollowMultiDOFJointTrajectoryControllerHandle
    : public moveit_simple_controller_manager::ActionBasedControllerHandle<moveit_msgs::ExecuteTrajectoryAction>
{
public:

  /**
   * @brief Constructor
   * @param name
   * @param action_ns
   */
  FollowMultiDOFJointTrajectoryControllerHandle(const std::string& name, const std::string& action_ns)
    : ActionBasedControllerHandle<moveit_msgs::ExecuteTrajectoryAction>(name, action_ns)
  {
  }

  /**
   * @brief Send ExecuteTrajectoryGoal message to action server
   * @param trajectory Trajectory to follow
   */
  bool sendTrajectory(const moveit_msgs::RobotTrajectory& trajectory) override
  {
    ROS_DEBUG_STREAM_NAMED("FollowMultiDOFJointTrajectoryController", "new trajectory to " << name_);

    if (!controller_action_client_)
      return false;

    if (!trajectory.joint_trajectory.points.empty())
    {
      ROS_WARN_NAMED("FollowMultiDOFJointTrajectoryController", "%s cannot execute trajectories(trajectory_msgs/JointTrajectory).", name_.c_str());
    }

    if (done_)
      ROS_DEBUG_STREAM_NAMED("FollowMultiDOFJointTrajectoryController", "sending trajectory to " << name_);
    else
      ROS_DEBUG_STREAM_NAMED("FollowMultiDOFJointTrajectoryController",
                             "sending continuation for the currently executed trajectory to " << name_);

    // Send ExecuteTrajectoryGoal message
    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory = trajectory;
    controller_action_client_->sendGoal(
        goal, boost::bind(&FollowMultiDOFJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
        boost::bind(&FollowMultiDOFJointTrajectoryControllerHandle::controllerActiveCallback, this),
        boost::bind(&FollowMultiDOFJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
    done_ = false;
    last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    return true;
  }

  /**
   * @brief Cancel trajecotry execution
   */
  bool cancelExecution() override
  {
    // do whatever is needed to cancel execution
    return true;
  }

  /**
   * @brief Wait trajectory execution
   * @param duration
   */
  bool waitForExecution(const ros::Duration& duration) override
  {
    // wait for the current execution to finish
    return true;
  }

protected:

  /**
   * @brief Called when server complete action
   * @param state
   * @param result
   */
  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const moveit_msgs::ExecuteTrajectoryResultConstPtr& result)
  {
    // Output custom error message for FollowJointTrajectoryResult if necessary
    if (result)
    {
      if(result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_INFO_NAMED("FollowMultiDOFTrajectoryContoller", "Execution Succeeded.");
      }
      else
      {
        ROS_ERROR("Returned Error Code %d", result->error_code.val);
        ROS_ERROR("For Detailse of Error Code, see moveit_msgs/MoveItErrorCodes.msg");
      }
    }
    else
    {
      ROS_WARN_STREAM("Controller " << name_ << ": no result returned");
    }

    finishControllerExecution(state);
  }

  /**
   * @brief Called when goal become active
   */
  void controllerActiveCallback()
  {
    ROS_DEBUG_STREAM_NAMED("FollowMultiDOFJointTrajectoryController", name_ << " started execution");
  }

  /**
   * @brief Called when feedback arrived from server
   * @param feedback
   */
  void controllerFeedbackCallback(const moveit_msgs::ExecuteTrajectoryFeedbackConstPtr& feedback)
  {
  }
};

}  // end namespace dronedoc

#endif // MOVEIT_PLUGINS_FOLLOW_MULTI_DOF_TRAJECTORY_CONTROLLER_HANDLE