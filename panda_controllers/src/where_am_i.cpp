// #include <franka_gripper/franka_gripper.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <franka/gripper_state.h>
// #include <franka_gripper/GraspAction.h>
// #include <franka_gripper/HomingAction.h>
// #include <franka_gripper/MoveAction.h>
// #include <franka_gripper/StopAction.h>
#include </opt/ros/noetic/include/control_msgs/GripperCommandActionGoal.h>
// #include </home/matteo/catkin_ws/devel/include/franka_gripper/GraspActionGoal.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <stdlib.h>

#include <geometry_msgs/PoseStamped.h>

#include <panda_controllers/DesiredTrajectory.h>
// #include <panda_controllers/DesiredTrajectory.h>
// #include <panda_controllers/cubeRef.h>
// #include "panda_controllers/utils/parsing_utilities.h"

#include <ros/ros.h>
// #include <gripper.h>

#include <sstream>
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Pose.h"

#include <Eigen/Dense>
#include <Vector3.hpp>
#include <Quaternion.hpp>
#include <Matrix3x3.hpp>

#define G 9.8182
#define PP_time 3 // Time for pick & place task
#define RATE_FREQ 500
#define MAX_V 1.7
#define MAX_A 13
#define DM false

using namespace std;

struct throwData
{
  Eigen::Vector3d xThrow;
  Eigen::Vector3d vThrow;
  Eigen::Vector3d aThrow;
  bool found;
  double theta;
};

struct angleData
{
  double theta;
  double v;
  bool found;
};

struct traj_struct
{
  Eigen::Vector3d pos_des;
  Eigen::Vector3d vel_des;
  Eigen::Vector3d acc_des;
  Quaternion or_des;
} traj;

typedef struct orient_struct
{
  double x;
  double y;
  double z;
  double w;
} msgQuat;

// New code
Eigen::Vector3d pos;
msgQuat orient;
bool pos_received = false;
// Previous one was:
// Eigen::Vector3d orient; //Be careful--> watch data type
// Orientation from ROS messages expressed in (x,y,z,w)
// See http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum)
{
  cout << "Caught signal " << signum << endl;
  // Terminate program
  exit(signum);
}

void poseCallback(
    const geometry_msgs::PoseStampedConstPtr &msg)
{

  pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  orient.x = msg->pose.orientation.x;
  orient.y = msg->pose.orientation.y;
  orient.z = msg->pose.orientation.z;
  orient.w = msg->pose.orientation.w;
  // cout << "Pose msg arrived:\n" << pos << endl; //Msg displayed
  pos_received = true;
}

void waitForPos()
{
  ros::Rate loop_rate(RATE_FREQ);
  pos_received = false;
  while (!pos_received)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void waitSec(double sec)
{
  ros::Time t_init;
  double t;
  t_init = ros::Time::now();
  t = (ros::Time::now() - t_init).toSec();
  while (t < sec)
  {
    t = (ros::Time::now() - t_init).toSec();
  }
}

int main(int argc, char **argv)
{

  // New code
  ros::init(argc, argv, "where_am_i");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredTrajectory>("/project_cartesian_throw/desired_trajectory", 1000);

  // ros::Publisher grip_cmd = node_handle.advertise<control_msgs::GripperCommandActionGoal>("/franka_gripper/gripper_action/goal",1);

  // ros::Publisher grip_grasp_cmd = node_handle.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal",10);

  // ros::Publisher pub_cube = node_handle.advertise<panda_controllers::cubeRef>("/qb_class/cube_ref",1);

  ros::Publisher pub_sh = node_handle.advertise<std_msgs::Float64>("/right_hand_v2s/synergy_command", 1);
  
  ros::Subscriber sub_cmd = node_handle.subscribe("/project_impedance_controller/franka_ee_pose", 1,
                                                  &poseCallback);                                                 
  ros::Subscriber sub_sim = node_handle.subscribe("/cartesian_impedance_example_controller/equilibrium_pose", 1,
                                                  &poseCallback);
  ros::Rate loop_rate(100);

  signal(SIGINT, signal_callback_handler);

  panda_controllers::DesiredTrajectory traj_msg;
  // control_msgs::GripperCommandActionGoal gripper_msg;
  // franka_gripper::GraspActionGoal gripper_grasp_msg;
  // End new code
  int res;

  cout << "Waiting for initial position" << endl;
  waitForPos();
  while (true)
  {
    waitForPos();
    cout << flush;
    res = system("clear");
    if(res == 0);
    cout << "I am here:" << endl;
    cout << "Position: " << pos.x() << "i + " << pos.y() << "j + " << pos.z() << "k " << endl;
    cout << "Orientation: " << orient.x << "i + " << orient.y << "j + " << orient.z << "k + " << orient.w << endl;
    waitSec(1);
  }
    
}

