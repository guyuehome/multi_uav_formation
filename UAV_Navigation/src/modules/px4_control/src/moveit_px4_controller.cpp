/**
 * @file drone_controller.cpp
 * @brief PX4 based UAV controller for moveit
 */

#include <vector>
#include <array>
#include <string>
#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <mavros_msgs/State.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <actionlib/server/simple_action_server.h>

class MoveitPx4Controller{

private:
    ros::NodeHandle nh_;
    //! Action name of ExecuteTrajectory
    std::string action_name_;
    //! Server of ExecuteTrajectoryAction
    actionlib::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction> as_;
    //! Feedback message of ExecuteTrajectoryAction
    moveit_msgs::ExecuteTrajectoryFeedback feedback_;
    //! Result message of ExecuteTrajectoryAction
    moveit_msgs::ExecuteTrajectoryResult result_;
    //! Position command publisher
    ros::Publisher cmd_pos_pub_;
    //! Subscriber of local position
    ros::Subscriber local_pos_sub_;
    //! Subscriber of UAV state
    ros::Subscriber state_sub_;
    //! Current pose of UAV
    geometry_msgs::PoseStamped current_pose_;
    //! Current state of UAV
    mavros_msgs::State current_state_;

public:

    /**
     * @brief Constructor
     * @param action_name Action name
     */
    MoveitPx4Controller(std::string action_name) :
        action_name_(action_name),
        as_(nh_, action_name, boost::bind(&MoveitPx4Controller::executeCb, this, _1), false)
    {
        as_.start();

        // Position command publisher setup
        std::string cmd_pos_topic;
        nh_.param<std::string>("mavros_setpoint_topic", cmd_pos_topic, "/mavros/setpoint_position/local");
        cmd_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cmd_pos_topic, 10);

        // Local position subscriber setup
        std::string local_pos_topic;
        nh_.param<std::string>("mavros_localpos_topic", local_pos_topic, "/mavros/local_position/pose");
        auto local_pos_cb = boost::bind(&MoveitPx4Controller::localPosCb, this, _1);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(local_pos_topic, 10, local_pos_cb);

        // UAV state subscriber setup
        std::string state_topic;
        nh_.param<std::string>("mavros_state_topic", state_topic, "/mavros/state");
        auto state_cb = boost::bind(&MoveitPx4Controller::stateCb, this, _1);
        state_sub_ = nh_.subscribe<mavros_msgs::State>(state_topic, 10, state_cb);
        ROS_INFO("Action server initialized.");
    };

    /**
     * @brief Destructor
     */
    ~MoveitPx4Controller()
    {
    };

private:

    /**
     * @brief Callback of ExecuteTrajectory action
     * @param goal Goal of ExecuteTrajectory action
     */
    void executeCb(const moveit_msgs::ExecuteTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO("Action received.");
        ros::Rate rate(20);

        std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory;
        trajectory = goal->trajectory.multi_dof_joint_trajectory.points;

        // Wait for connection
        ROS_INFO("Waiting for FCU connection...");
        while (ros::ok() and not current_state_.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("FCU connection established.");

        // Position command need to be published to switch mode to OFFBOARD
        for(int i=0; i<10; ++i)
        {
            cmd_pos_pub_.publish(getPoseFromTrajectory(trajectory.front()));
        }


        for(int i=0; i < trajectory.size()-1; ++i)
        {
            ROS_INFO("Moving to waypoint No. %d", i);

            // Send feedback (progress of path)
            feedback_.state = std::to_string(i);
            as_.publishFeedback(feedback_);

            // Get PoseStamped message from MultiDOFJointTrajectoryPoint
            geometry_msgs::PoseStamped start = getPoseFromTrajectory(trajectory.at(i));
            geometry_msgs::PoseStamped goal = getPoseFromTrajectory(trajectory.at(i+1));
            // Get interpolated path from two waypoints
            std::vector<geometry_msgs::PoseStamped> path = getBilinearPath(start, goal);

            // Publish all interpolated points
            for(auto pose: path)
            {
                // Publish same message till drone arrives to local goal
                while(not isGoalReached(pose) and not as_.isPreemptRequested())
                {
                    cmd_pos_pub_.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                }

                // Exit loop if new action arrived
                if(as_.isPreemptRequested() or not ros::ok())
                {
                    result_.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
                    as_.setPreempted();
                    ROS_INFO("Action preempted.");
                    break;
                }
            }

            // Exit loop if new action arrived
            if(as_.isPreemptRequested() or not ros::ok())
            {
                result_.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
                as_.setPreempted();
                ROS_INFO("Action preempted.");
                break;
            }
            rate.sleep();
        }

        // Success
        result_.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        as_.setSucceeded(result_);
        ROS_INFO("Action completed.");
    }

    /**
     * @brief Return interpolated path using bilinear interpolation from two waypoints
     * @param start Start waypoint
     * @param goal Goal waypoint
     * @param step Step size of interpolation
     * @return Vector of interpolated path
     */
    std::vector<geometry_msgs::PoseStamped> getBilinearPath(const geometry_msgs::PoseStamped &start,
                                                            const geometry_msgs::PoseStamped &goal,
                                                            const double step=0.05)
    {
        std::vector<geometry_msgs::PoseStamped> bilinear_path;

        // Store x-y and x-z coordinates of start point
        std::array<double, 2> start_xy = {start.pose.position.x, start.pose.position.y};
        std::array<double, 2> start_xz = {start.pose.position.x, start.pose.position.z};
        // Store x-y and x-z coordinates of goal point
        std::array<double, 2> goal_xy = {goal.pose.position.x, goal.pose.position.y};
        std::array<double, 2> goal_xz = {goal.pose.position.x, goal.pose.position.z};

        // x-y and x-z coordinates of interpolated points
        std::array<std::vector<double>, 2> points_xy = linearInterp(start_xy, goal_xy, step);
        std::array<std::vector<double>, 2> points_xz = linearInterp(start_xz, goal_xz, step);

        // Number of generated points by interpolation
        int num_points = points_xy.at(0).size();

        try
        {
            // Generate PoseStamped message from std::array
            for(int i=0; i<num_points; ++i)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.orientation = start.pose.orientation;
                pose.pose.position.x = points_xy.front().at(i);
                pose.pose.position.y = points_xy.back().at(i);
                pose.pose.position.z = points_xz.back().at(i);

                bilinear_path.push_back(pose);
            }
        }
        catch (std::out_of_range &ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        return bilinear_path;
    }

    /**
     * @brief Perform linear interpolation
     * @param p1 First point
     * @param p2 Second point
     * @param step Step size of interpolation
     * @return Array of interpolated points in the shape of [x-points, y-points]
     */
    std::array<std::vector<double>, 2> linearInterp(const std::array<double, 2> &p1,
                                                    const std::array<double, 2> &p2, const double step)
    {
        // Gradient
        double a = (p1.at(1) - p2.at(1)) / (p1.at(0) - p2.at(0));
        // Intercept
        double b = p1.at(1) - a*p1.at(0);

        // Number of steps
        int num_steps = std::floor((p2.at(0) - p1.at(0))/step);

        // Initialize container for interpolated points
        std::vector<double> points_x(num_steps+1);

        // Set interpolated points
        points_x.front() = p1.at(0);
        for(int i=1; i<num_steps; ++i)
        {
            points_x.at(i) = step * i + p1.at(0);
        }
        points_x.back() = p2.at(0);

        // Initialize container for interpolated points
        std::vector<double> points_y(num_steps+1);

        // Set interpolated points
        points_y.front() = p1.at(1);
        for(int i=1; i<num_steps; ++i)
        {
            points_y.at(i) = a*(p1.at(0) + i*step) + b;
        }
        points_y.back() = p2.at(1);

        // Initialize container for vector of points
        std::array<std::vector<double>, 2> points;
        points.front() = points_x;
        points.back() = points_y;

        return points;
    }

    /**
     * @brief Convert MultiDOFJointTrajectoryPoint message to PoseStamped message
     * @param trajectory_pt MultiDOFJointTrajectoryPoint message
     * @return Pose converted from Trajectory msg
     */
    geometry_msgs::PoseStamped getPoseFromTrajectory(const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_pt)
    {
        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = trajectory_pt.transforms.front().translation.x;
        pose.pose.position.y = trajectory_pt.transforms.front().translation.y;
        pose.pose.position.z = trajectory_pt.transforms.front().translation.z;
        pose.pose.orientation = trajectory_pt.transforms.front().rotation;

        return pose;
    }

    /**
     * @brief Returns true if drone is reached goal
     * @param goal
     * @param tolerance Consider drone reached goal if drone is within the circle with diameter of this value
     * @return True if drone is reched goal, false if else
     */
    inline bool isGoalReached(const geometry_msgs::PoseStamped &goal, const double tolerance=0.1)
    {
        double error = std::sqrt(std::pow(goal.pose.position.x - current_pose_.pose.position.x, 2)
                                 + std::pow(goal.pose.position.y - current_pose_.pose.position.y, 2)
                                 + std::pow(goal.pose.position.z - current_pose_.pose.position.z, 2));
        return error < tolerance ? true : false;
    }

    /**
     * @brief Callback for local position subscriber
     * @msg Incoming message
     */
    void localPosCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
    }

    /**
     * @brief Callback for state subscriber
     * @msg Incoming message
     */
    void stateCb(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state_ = *msg;
    }

};

int main(int argv, char **argc)
{
    ros::init(argv, argc, "moveit_px4_controller");

    MoveitPx4Controller controller("iris_group_controller/follow_multi_dof_joint_trajectory");
    ros::spin();

    return 0;
}
