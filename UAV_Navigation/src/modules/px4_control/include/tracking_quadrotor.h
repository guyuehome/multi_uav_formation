#include <ros/ros.h>
#include <iostream>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include "offboard_control.h"
#include "px4_control_cfg.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
using namespace std;
using namespace Eigen;
class PX4Tracking {
 public:
    /**
     *默认构造函数
     */
    PX4Tracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    /**
     * 析构函数
     */
    ~PX4Tracking();
    void Initialize();
   OffboardControl OffboardControl_;
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer cmdloop_timer_;
  void CmdLoopCallback(const ros::TimerEvent& event);
  void TrackingStateUpdate();
  void ArPoseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void Px4PosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void Px4StateCallback(const mavros_msgs::State::ConstPtr& msg);
  Eigen::Vector3d TrackingPidProcess(Eigen::Vector3d &currentPos,Eigen::Vector3d &expectPos);
  Eigen::Vector3d temp_pos_drone;
  Eigen::Vector3d posxyz_target;//期望飞机的空间位置
  Eigen::Vector3d  ar_pose_;  //二维码相对飞机位置
  Eigen::Vector3d  px4_pose_; //接收来自飞控的飞机位置 
  Eigen::Vector3d desire_pose_;//期望的飞机相对二维码的位置
  mavros_msgs::State px4_state_;//飞机的状态
  mavros_msgs::SetMode mode_cmd_;
  float search_alt_;
  float markers_id_;//需要检测到的二维码，默认是4
  bool detect_state;//是否检测到二维码标志位
  Eigen::Vector3d desire_vel_;
	Eigen::Vector3d desire_yzVel_;
	float desire_yawVel_;
  S_PID s_PidY,s_PidZ,s_PidYaw;
  S_PID_ITEM s_PidItemY;
  S_PID_ITEM s_PidItemZ;
  S_PID_ITEM s_PidItemYaw;
  enum
 {
  WAITING,		//等待offboard模式
  CHECKING,		//检查飞机状态
  PREPARE,		//起飞到指定高度
  SEARCH,		  //搜索
  TRACKING,	  //检测到二维码，开始跟踪
  TRACKOVER,	//结束		
}TrackingState = WAITING;//初始状态WAITING

  ros::Subscriber ar_pose_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_mode_client_;
};
