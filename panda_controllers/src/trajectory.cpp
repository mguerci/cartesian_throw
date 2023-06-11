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
// End new code

double **getMatrix(string fileName, int n, int m) // Works
{
  ifstream myfile;
  myfile.open(fileName);
  double **MatrixPointer = NULL;
  MatrixPointer = new double *[n];
  // Allocating memory
  for (int i = 0; i < n; i++)
    MatrixPointer[i] = new double[m];

  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < m; j++)
    {
      myfile >> MatrixPointer[i][j];
      // cout << MatrixPointer[i][j] << " ";
    }
    // cout << endl;
  }

  return MatrixPointer;
}

int sign(double val)
{
  if (val > 0)
    return 1;
  else if (val < 0)
    return -1;

  return 0;
}

angleData computeAngle(double l, double zTh, double g, double xTh, double xGr)
{
  double delta_z, delta_x, F;
  double vMin = 100000000;
  double theta_min, theta_start = M_PI_4;
  double tr = 0.01;
  angleData result;
  result.found = false;

  delta_z = zTh + l;
  delta_x = xGr - xTh;

  for (double theta = -M_PI / 3; theta <= M_PI / 3; theta += 0.1)
  {
    for (double v = 0.1; v <= 5; v += 0.05)
    {
      F = -delta_x + ((pow(v, 2)) / (2 * g)) * (sin(2 * theta) + 2 * cos(theta) * sqrt((pow(sin(theta), 2)) + (2 * g * delta_z) / (pow(v, 2))));
      if (abs(F) <= tr)
      {
        if (abs(v) < vMin)
        {
          result.found = true;
          result.theta = theta;
          result.v = v;
        }
      }
    }
  }
  return result;
}

throwData computeThrow(Eigen::Vector3d xGround, double **mat, int n, int m, double marg)
{
  double vMin = 100000000, beta, dTh, zTh, Dx, Dz, v, tFl, l;
  Eigen::Vector3d xThrow;
  xThrow.x() = 0;
  xThrow.y() = 0;
  xThrow.z() = 0;
  Eigen::Vector3d vThrow;
  vThrow.x() = 0;
  vThrow.y() = 0;
  vThrow.z() = 0;
  Eigen::Vector3d aThrow;
  aThrow.x() = 0;
  aThrow.y() = 0;
  aThrow.z() = 0;
  bool found = false;
  throwData result;
  angleData ad;

  char temp;

  // cout << "Computing throw data for X_ground = \n"  << xGround << endl;
  // cin >> temp;
  // xGround ok
  l = -xGround.z();
  beta = atan2(xGround.y(), xGround.x());

  // for (int i = 0; i < m; i++)
  // {
  //   zTh = mat[1][i];
  //   if (zTh>=0.5)
  //   {
  //       dTh = mat[0][i];
  //       // cout << "X = " << dTh << " ; Z = " << zTh << endl;
  //       // while (cin.get() != '\n');

  //       Dx = sqrt(pow(xGround.x(), 2) + pow(xGround.y(), 2)) + marg;
  //       ad = computeAngle(l, zTh, G, dTh, Dx);
  //       if(ad.found)
  //       {
  //         if(ad.v < vMin)
  //         {
  //           // cout << "Found result\n" << endl;
  //           // cout << "X = " << dTh << " ; Z = " << zTh << " ; V = " << ad.v << endl;
  //           // cin >> carlo;
  //           vMin = ad.v;
  //           result.theta = ad.theta;
  //           result.found = true;
  //           xThrow.x() = (dTh - marg * sign(dTh)) * cos(beta);
  //           xThrow.y() = (dTh - marg * sign(dTh)) * sin(beta);
  //           xThrow.z() = zTh;
  //           vThrow.x() = vMin * cos(beta) * cos(ad.theta);
  //           vThrow.y() = vMin * sin(beta) * cos(ad.theta);
  //           vThrow.z() = vMin * sin(ad.theta);
  //         }
  //       }

  //   }

  // }

  for (int i = 0; i < m; i++)
  {
    zTh = mat[1][i];
    if (zTh >= 0.5)
    {
      dTh = mat[0][i];
      Dx = sqrt(pow(xGround.x(), 2) + pow(xGround.y(), 2)) - dTh + marg;
      Dz = xGround.z() - zTh;
      v = sqrt((pow(Dx, 2) * G) / (2 * (Dx - Dz)));

      if (!isnan(v) && v < vMin)
      {
        tFl = Dx / v;
        found = true;
        vMin = v;
        xThrow.x() = (dTh - marg * sign(dTh)) * cos(beta);
        xThrow.y() = (dTh - marg * sign(dTh)) * sin(beta);
        xThrow.z() = zTh;

        vThrow.x() = v * cos(beta);
        vThrow.y() = v * sin(beta);
        vThrow.z() = v;
      }
    }
  }
  result.found = found;
  result.xThrow = xThrow;
  result.vThrow = vThrow;
  result.aThrow = aThrow;
  return result;
}

void demo_inf_XY(Eigen::Vector3d pos_i, double t)
{
  Eigen::Vector3d tmp;
  double beta = 1;
  tmp << sin(t / beta) / 8, sin(t / 2 * beta) / 4, 0;
  traj.pos_des << pos_i + tmp;
  traj.vel_des << cos(t / beta) / (8 * beta), cos(t / 2 * beta) / (8 * beta), 0;
  traj.acc_des << -sin(t / beta) / (8 * pow(beta, 2)), -sin(t / 2 * beta) / (16 * pow(beta, 2)), 0;
}

void interpolator_pos(Eigen::Vector3d pos_i, Eigen::Vector3d pos_f,
                      double tf, double t)
{

  traj.pos_des << pos_i + (pos_i - pos_f) * (15 * pow((t / tf), 4) - 6 * pow((t / tf), 5) - 10 * pow((t / tf), 3));
  traj.vel_des << (pos_i - pos_f) * (60 * (pow(t, 3) / pow(tf, 4)) - 30 * (pow(t, 4) / pow(tf, 5)) - 30 * (pow(t, 2) / pow(tf, 3)));
  traj.acc_des << (pos_i - pos_f) * (180 * (pow(t, 2) / pow(tf, 4)) - 120 * (pow(t, 3) / pow(tf, 5)) - 60 * (t / pow(tf, 3)));
}

void interpolator_posSpeed(Eigen::Vector3d pos_i, Eigen::Vector3d vel_i,
                           double tf, double t)
{
  // Used to stop after throw
  traj.pos_des << pos_i + vel_i * t + vel_i * (-0.5 * pow(t, 2) / tf);
  traj.vel_des << vel_i * (1 - (t / tf));
  traj.acc_des << vel_i * (-1 / tf);
}

Quaternion normalizeQuat(Quaternion q)
{
  Quaternion n;
  double norm;
  norm = sqrt(pow(q.X, 2) + pow(q.Y, 2) + pow(q.Z, 2) + pow(q.W, 2));
  n.X = q.X / norm;
  n.Y = q.Y / norm;
  n.Z = q.Z / norm;
  n.W = q.W / norm;
  return n;
}

double phi(Quaternion q)
{
  double phi;
  q = normalizeQuat(q);
  phi = atan2(2 * (q.W * q.Z + q.X * q.Y), 1 - 2 * (pow(q.Y, 2) + pow(q.Z, 2)));
  return phi;
}

double theta(Quaternion q)
{
  double theta;
  q = normalizeQuat(q);
  theta = -M_PI / 2 + 2 * atan2(sqrt(1 + 2 * (q.W * q.Y - q.X * q.Z)), sqrt(1 - 2 * (q.W * q.Y - q.X * q.Z)));
  return theta;
}

double psi(Quaternion q)
{
  double psi;
  q = normalizeQuat(q);
  psi = atan2(2 * (q.W * q.X + q.Y * q.Z), 1 - 2 * (pow(q.X, 2) + pow(q.Y, 2)));
  return psi;
}

bool isClose(Eigen::Vector3d p1, Eigen::Vector3d p2, double marg)
{
  if (abs(p1.x() - p2.x()) < marg)
  {
    if (abs(p1.y() - p2.y()) < marg)
    {
      if (abs(p1.z() - p2.z()) < marg)
      {
        return true;
      }
    }
  }
  return false;
}

control_msgs::GripperCommandActionGoal gripMove(double ratio, double max_effort)
{
  double position;
  if (ratio >= 0 && ratio <= 1)
    position = 0.04 * ratio;
  else
  {
    if (ratio < 0)
      position = 0;
    else
      position = 0.04;
  }

  control_msgs::GripperCommandActionGoal gripper_msg;

  gripper_msg.header.frame_id = "";
  gripper_msg.header.seq = 0;
  gripper_msg.header.stamp.sec = 0;
  gripper_msg.header.stamp.nsec = 0;

  gripper_msg.goal_id.id = "";
  gripper_msg.goal_id.stamp.sec = 0;
  gripper_msg.goal_id.stamp.nsec = 0;

  gripper_msg.goal.command.max_effort = max_effort;
  gripper_msg.goal.command.position = position;

  return gripper_msg;
}

Eigen::Matrix<double, 3, 3> getRM(double angle, char ax)
{
  Eigen::Matrix<double, 3, 3> R;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; i < 3; i++)
    {
      R(i, j) = 0;
    }
  }
  switch (ax)
  {
  case 'x':
    R(0, 0) = 1;
    R(2, 1) = sin(angle);
    R(1, 1) = cos(angle);
    R(1, 2) = -sin(angle);
    R(2, 2) = cos(angle);
    break;

  case 'y':

    R(0, 0) = cos(angle);
    R(0, 2) = sin(angle);
    R(1, 1) = 1;
    R(2, 0) = -sin(angle);
    R(2, 2) = cos(angle);
    break;

  case 'z':
    R(0, 0) = cos(angle);
    R(0, 1) = -sin(angle);
    R(1, 0) = sin(angle);
    R(1, 1) = cos(angle);
    R(2, 2) = 1;
    break;

  default:
    cout << "Non valid axis" << endl;
    break;
  }

  return R;
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

void move_EE(Eigen::Vector3d posStart, Eigen::Vector3d posEnd, Quaternion curr_or, ros::Publisher pub_cmd)
{
  ros::Time t_init;
  double t, tr;
  ros::Rate loop_rate(RATE_FREQ);
  panda_controllers::DesiredTrajectory traj_msg;
  t_init = ros::Time::now();
  waitForPos();
  while (!isClose(pos, posEnd, 0.01))
  {
    t = (ros::Time::now() - t_init).toSec();
    tr = t;
    if (t > PP_time)
    {
      t = PP_time;
    }

    if (tr > 2 * PP_time)
    {
      break;
    }

    interpolator_pos(posStart, posEnd, PP_time, t);

    // Now message has to be sent
    traj_msg.header.stamp = ros::Time::now();

    traj_msg.pose.position.x = traj.pos_des.x();
    traj_msg.pose.position.y = traj.pos_des.y();
    traj_msg.pose.position.z = traj.pos_des.z();

    traj_msg.velocity.position.x = traj.vel_des.x();
    traj_msg.velocity.position.y = traj.vel_des.y();
    traj_msg.velocity.position.z = traj.vel_des.z();

    traj_msg.acceleration.position.x = traj.acc_des.x();
    traj_msg.acceleration.position.y = traj.acc_des.y();
    traj_msg.acceleration.position.z = traj.acc_des.z();

    // // Sending initial orientation
    // traj_msg.pose.orientation.x = quatStart.X;
    // traj_msg.pose.orientation.y = quatStart.Y;
    // traj_msg.pose.orientation.z = quatStart.Z;
    // traj_msg.pose.orientation.w = quatStart.W;

    // Sending hand orientation
    traj_msg.pose.orientation.x = curr_or.X;
    traj_msg.pose.orientation.y = curr_or.Y;
    traj_msg.pose.orientation.z = curr_or.Z;
    traj_msg.pose.orientation.w = curr_or.W;

    pub_cmd.publish(traj_msg);
    waitForPos();
  }
}

void rotate_EE(Quaternion quatStart, Quaternion quatEnd, Eigen::Vector3d curr_pos, ros::Publisher pub_cmd)
{
  ros::Time t_init;
  double t;
  ros::Rate loop_rate(RATE_FREQ);
  panda_controllers::DesiredTrajectory traj_msg;
  t_init = ros::Time::now();
  t = (ros::Time::now() - t_init).toSec();
  while (t < PP_time)
  {
    Quaternion quatDes = Quaternion::Slerp(quatStart, quatEnd, t / PP_time);

    // Now message has to be sent
    traj_msg.header.stamp = ros::Time::now();

    traj_msg.pose.position.x = curr_pos.x();
    traj_msg.pose.position.y = curr_pos.y();
    traj_msg.pose.position.z = curr_pos.z();

    traj_msg.velocity.position.x = 0;
    traj_msg.velocity.position.y = 0;
    traj_msg.velocity.position.z = 0;

    traj_msg.acceleration.position.x = 0;
    traj_msg.acceleration.position.y = 0;
    traj_msg.acceleration.position.z = 0;

    traj_msg.pose.orientation.x = quatDes.X;
    traj_msg.pose.orientation.y = quatDes.Y;
    traj_msg.pose.orientation.z = quatDes.Z;
    traj_msg.pose.orientation.w = quatDes.W;

    pub_cmd.publish(traj_msg);
    loop_rate.sleep();

    t = (ros::Time::now() - t_init).toSec();
  }
}

void moveRotate_EE(Quaternion quatStart, Quaternion quatEnd, Eigen::Vector3d posStart, Eigen::Vector3d posEnd, ros::Publisher pub_cmd)
{
  ros::Time t_init;
  double t, tr;
  ros::Rate loop_rate(RATE_FREQ);
  panda_controllers::DesiredTrajectory traj_msg;
  t_init = ros::Time::now();
  t = (ros::Time::now() - t_init).toSec();
  waitForPos();
  while (!isClose(pos, posEnd, 0.01))
  {
    t = (ros::Time::now() - t_init).toSec();
    tr = t;
    if (t > PP_time)
    {
      t = PP_time;
    }

    if (tr > 2 * PP_time)
    {
      break;
    }

    interpolator_pos(posStart, posEnd, PP_time, t);
    Quaternion quatDes = Quaternion::Slerp(quatStart, quatEnd, t / PP_time);

    // Now message has to be sent
    traj_msg.header.stamp = ros::Time::now();

    traj_msg.pose.position.x = traj.pos_des.x();
    traj_msg.pose.position.y = traj.pos_des.y();
    traj_msg.pose.position.z = traj.pos_des.z();

    traj_msg.velocity.position.x = traj.vel_des.x();
    traj_msg.velocity.position.y = traj.vel_des.y();
    traj_msg.velocity.position.z = traj.vel_des.z();

    traj_msg.acceleration.position.x = traj.acc_des.x();
    traj_msg.acceleration.position.y = traj.acc_des.y();
    traj_msg.acceleration.position.z = traj.acc_des.z();

    // // Sending initial orientation
    // traj_msg.pose.orientation.x = quatStart.X;
    // traj_msg.pose.orientation.y = quatStart.Y;
    // traj_msg.pose.orientation.z = quatStart.Z;
    // traj_msg.pose.orientation.w = quatStart.W;

    // Sending hand orientation
    traj_msg.pose.orientation.x = quatDes.X;
    traj_msg.pose.orientation.y = quatDes.Y;
    traj_msg.pose.orientation.z = quatDes.Z;
    traj_msg.pose.orientation.w = quatDes.W;
    pub_cmd.publish(traj_msg);
    waitForPos();
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
  ros::init(argc, argv, "trajectory");

  ros::NodeHandle node_handle;

  ros::Publisher pub_cmd = node_handle.advertise<panda_controllers::DesiredTrajectory>("/project_cartesian_throw/desired_trajectory", 1000);

  // ros::Publisher grip_cmd = node_handle.advertise<control_msgs::GripperCommandActionGoal>("/franka_gripper/gripper_action/goal",1);

  // ros::Publisher grip_grasp_cmd = node_handle.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal",10);

  // ros::Publisher pub_cube = node_handle.advertise<panda_controllers::cubeRef>("/qb_class/cube_ref",1);

  ros::Publisher pub_sh = node_handle.advertise<std_msgs::Float64>("/right_hand_v2s/synergy_command", 1);

  ros::Subscriber sub_cmd = node_handle.subscribe("/project_impedance_controller/franka_ee_pose", 1,
                                                  &poseCallback);

  ros::Rate loop_rate(100);

  signal(SIGINT, signal_callback_handler);

  panda_controllers::DesiredTrajectory traj_msg;
  // control_msgs::GripperCommandActionGoal gripper_msg;
  // franka_gripper::GraspActionGoal gripper_grasp_msg;
  // End new code

  Eigen::Vector3d pos_f;
  Eigen::Vector3d pos_init;
  Eigen::Vector3d or_init;
  Eigen::Vector3d obj_init;
  Eigen::Vector3d pos_bar;
  Eigen::Vector3d pos_target;
  Eigen::Vector3d pos_startThrow;
  // Eigen::Vector3d pos_reset;
  // msgQuat orient_reset;

  // pos_reset << 0.31, 0, 0.5;
  // orient_reset.x = 1;
  // orient_reset.y = 0;
  // orient_reset.z = 0;
  // orient_reset.w = 0;

  pos_bar.x() = 0.45;
  pos_bar.y() = 0.2;
  pos_bar.z() = 0.7;
  throwData TR;
  int n, m;
  double **mat = NULL;
  double tJerk, tThrow, tEnd;
  double th;
  double thMode;
  double debug;
  // node_handle.getParam("th", th);
  // cout << th << endl;
  bool firstSwitch = true;
  bool demo = DM;
  double marg;
  throwData throwValues;
  std_msgs::Float64 hand_msg;

  // franka::Gripper gripper("$robot_ip");

  // std::string action(argv[2]);
  // if (action == "dem") {
  //     demo = true;
  // } else if (action == "") {
  //     demo = false;
  // } else {
  //     // Report invalid argument
  // }

  n = 2;
  m = 110;
  mat = getMatrix("/home/matteo/catkin_ws/src/panda_controllers/Matrix.txt", n, m);
  // mat = getMatrix("..//Matrix.txt", n, m);

  // for (int i = 0; i < n; i++)
  // {
  //   for (int j = 0; j < m; j++)
  //   {
  //     cout << mat[i][j] << " ";
  //   }
  //   cout << endl;
  // }

  // Obtaining initial position from topic
  // waitSec(10);
  cout << "Waiting for initial position" << endl;
  waitForPos();
  // while (true)
  // {
  //   waitForPos();
  //   cout << "Orientation: " << orient.x << "i + " << orient.y << "j + " << orient.z << "k + " << orient.w << endl;
  // }
  // cout << "Orientation: " << orient.x << " "<< orient.y << " "<< orient.z << " "<< orient.w << endl;
  // From message
  pos_init = pos;

  ros::Time t_init;
  double t;

  waitForPos();
  // From message
  pos_init = pos;

  // cout << "initial position received: \n"
  //      << pos_init << endl;
  // cout << "initial orientation received: " << orient.x << " " << orient.y << " " << orient.z << " " << orient.w << endl;

  Quaternion quatReset = Quaternion(0.999997, -0.000151483, 0.0010959, 3.50181e-05);
  Quaternion quatStart = Quaternion(orient.x, orient.y, orient.z, orient.w);
  // quatStart = quatReset;

  // New: with real time
  double ratio;
  int nM = 0;

  if (demo)
  {
    waitForPos();
    move_EE(pos, pos_bar, quatStart, pub_cmd);
    t_init = ros::Time::now();
    t = (ros::Time::now() - t_init).toSec();
    while (ros::ok())
    {
      traj.or_des = quatStart;
      demo_inf_XY(pos_init, t);
      // Now message has to be sent
      traj_msg.header.stamp = ros::Time::now();

      traj_msg.pose.position.x = traj.pos_des.x();
      traj_msg.pose.position.y = traj.pos_des.y();
      traj_msg.pose.position.z = traj.pos_des.z();

      traj_msg.velocity.position.x = traj.vel_des.x();
      traj_msg.velocity.position.y = traj.vel_des.y();
      traj_msg.velocity.position.z = traj.vel_des.z();

      traj_msg.acceleration.position.x = traj.acc_des.x();
      traj_msg.acceleration.position.y = traj.acc_des.y();
      traj_msg.acceleration.position.z = traj.acc_des.z();

      // Sending initial orientation
      traj_msg.pose.orientation.x = quatStart.X;
      traj_msg.pose.orientation.y = quatStart.Y;
      traj_msg.pose.orientation.z = quatStart.Z;
      traj_msg.pose.orientation.w = quatStart.W;

      pub_cmd.publish(traj_msg);

      nM++;
      loop_rate.sleep();
      t = (ros::Time::now() - t_init).toSec();
      // cout << "Elapsed time: " << t << endl;
      pos_received = false;
      while (!pos_received)
      {
        ros::spinOnce();
        loop_rate.sleep();
      }

      // cout << "Orientation received: " << orient.x << " " << orient.y << " " << orient.z << " " << orient.w << endl;
    }
  }
  else
  {
    double time_adv, scale;
    // waitSec(10);
    hand_msg.data = 0.0;
    pub_sh.publish(hand_msg);
    waitSec(3);
    cout << "Hand full open" << endl;

    Quaternion quatHand;
    quatHand.X = -0.6533;
    quatHand.Y = -0.2706;
    quatHand.Z = -0.2706;
    quatHand.W = 0.6533;
    // quatHand.X = 0.26984;
    // quatHand.Y = -0.643745;
    // quatHand.Z = 0.649614;
    // quatHand.W = 0.30124;

    // waitForPos();
    // move_EE(pos, pos_bar, quatStart, pub_cmd);
    // waitForPos();
    // rotate_EE(quatStart, quatHand, pos, pub_cmd);
    waitForPos();
    moveRotate_EE(quatStart, quatHand, pos, pos_bar, pub_cmd);

    // obj_init.x() = 0.38;
    // obj_init.y() = -0.26;
    // obj_init.z() = 0.02;

    obj_init.x() = 0.38;
    obj_init.y() = -0.26;
    obj_init.z() = 0.06;

    // REAL VALUES
    obj_init.x() = 0.55;
    obj_init.y() = -0.35;
    obj_init.z() = 0.05;

    // obj_init.x() = -0.426;
    // obj_init.y() = -0.02;
    // obj_init.z() = 0.485;

    // pos_target.x() = 0 ;
    // pos_target.y() = 0.5;
    // pos_target.z() = obj_init.z();
    // moveRotate_EE(quatStart, quatHand, pos, pos_target, pub_cmd);
    // cin >> scale;

    pos_target = obj_init;
    pos_target.z() = obj_init.z() + 0.13;
    waitForPos();
    move_EE(pos, pos_target, quatHand, pub_cmd);
    waitForPos();
    move_EE(pos, obj_init, quatHand, pub_cmd);
    cout << "Here comes the hand" << endl;
    // cout << "Waiting for enter to close" << endl;
    // while (cin.get() != '\n');
    hand_msg.data = 0.9;
    pub_sh.publish(hand_msg);
    cout << "Hand closing" << endl;
    waitSec(3);
    cout << "Hand closed" << endl;

    // cout << "Going to q_bar position" << endl;
    // waitForPos();
    // move_EE(pos, pos_bar, quatHand, pub_cmd);

    pos_startThrow.x() = 0.4;
    pos_startThrow.y() = -0.3;
    pos_startThrow.z() = 0.37;
    cout << "Going to start throw position" << endl;
    waitForPos();
    // move_EE(pos, pos_startThrow, quatHand, pub_cmd);
    quatStart = quatHand;

    // quatHand.X = 0.208214;
    // quatHand.Y = 0.00316372;
    // quatHand.Z = 0.298056;
    // quatHand.W = -0.931559;


    // // Quaternione per lancio su x
    // quatHand.X = -0.262875;
    // quatHand.Y = 0.398214;
    // quatHand.Z = -0.426742;
    // quatHand.W = 0.768254;

    // Quaternione per lancio obliquo
    quatHand.X = -0.2484;
    quatHand.Y = 0.4319;
    quatHand.Z = 0.2944;
    quatHand.W = 0.8169;
    moveRotate_EE(quatStart, quatHand, pos, pos_startThrow, pub_cmd);

    // Rotating hand in throw orientation
    quatStart = quatHand;
    // cout << "Select throw mode: ";
    // cin >> thMode;
    thMode = 3;
    // if ((thMode != 1) && (thMode != 2))
    // {
    //   thMode = 1;
    // }

    if (thMode == 1)
    {
      // For x-throw
      marg = 0.2;
      quatHand.X = 0.208214;
      quatHand.Y = 0.00316372;
      quatHand.Z = 0.298056;
      quatHand.W = -0.931559;

      // X-throw
      pos_f.x() = 1.6;
      pos_f.y() = 1;
      pos_f.z() = 0;

      // X-throw
      scale = 2.8;
      tJerk = 0.2;
      time_adv = 0.6;
    }
    else if (thMode == 2) // Non va più
    {
      // For y-throw
      marg = 0.2;
      quatHand.X = -0.15907;
      quatHand.Y = 0.082477;
      quatHand.Z = 0.46932;
      quatHand.W = 0.86466;

      // Y-throw
      pos_f.x() = 0.4;
      pos_f.y() = 1.6;
      pos_f.z() = 0;

      scale = 2.8;
      tJerk = 0.4;
      time_adv = 0.4;
    }
    else if (thMode == 3)
    {
      // For x-throw
      marg = 0.20;
      // Quaternione per lancio dritto su x
      quatHand.X = -0.262875;
      quatHand.Y = 0.398214;
      quatHand.Z = -0.426742;
      quatHand.W = 0.768254;

      // Quaternione per lancio obliquo
      quatHand.X = -0.2484;
      quatHand.Y = 0.4319;
      quatHand.Z = 0.2944;
      quatHand.W = 0.8169;

      cout << "Final position of the object (x,y,z): " << endl;
      cin >> pos_f.x();
      cin >> pos_f.y();
      cin >> pos_f.z();
      // X-throw
      // pos_f.x() = 1;
      // pos_f.y() = 0.1;
      // pos_f.z() = 0;

      // X-throw
      cout << "Time for minJerk: ";
      cin >> tJerk;
      cout << "Hand cmd time advance ";
      cin >> time_adv;
      cout << "Scale factor: ";
      cin >> scale;
      // scale = 1;
      // tJerk = 0.2;
      // time_adv = 0.6;
    }
    else if (thMode == 4)
    {
      marg = 0.2;
      quatHand.X = 0.208214;
      quatHand.Y = 0.00316372;
      quatHand.Z = 0.298056;
      quatHand.W = -0.931559;

      pos_f.x() = 2;
      pos_f.y() = 0.4;
      pos_f.z() = 0;

      scale = 1;
      tJerk = 0.4;
      time_adv = 0.1;
    }

    // waitForPos();
    // rotate_EE(quatStart, quatHand, pos, pub_cmd);

    waitForPos();
    pos_init = pos;
    // cout << "Final position of the object (x,y,z): " << endl;
    // cin >> pos_f.x();
    // cin >> pos_f.y();
    // cin >> pos_f.z();

    bool fsHand = true;
    double modV, modA;

    // cout << "Time for minJerk: ";
    // cin >> tJerk;
    // cout << "Hand cmd time advance ";
    // cin >> time_adv;
    // cout << "Scale factor: ";
    // cin >> scale;

    tThrow = tJerk * scale;

    // th = 0.12; // Time to stop
    // th = 0.08;
    th = 0.3;
    tEnd = tThrow + th;

    TR = computeThrow(pos_f, mat, n, m, marg);
    if (!TR.found)
    {
      ROS_ERROR_STREAM("Trajectory: Could not compute throw parameters ");
      return false;
    }

    double beta, phi, theta, eta;

    // New attempt
    Eigen::Matrix<double, 3, 3> Rx;
    // First rotation: x-axis--> -Pi/2

    Eigen::Matrix<double, 3, 3> Rz;
    Eigen::Matrix<double, 3, 3> Ry;
    Eigen::Matrix<double, 3, 3> R_tot;
    // First rotation: beta on z-axis
    beta = atan2(pos_f.y(), pos_f.x());
    // Second rotation: phi on "new" y-axis
    // phi=-pi/4; per versione di lancio a 45°
    phi = -TR.theta; //  per versione nuova

    // // Computing parameters
    Eigen::Matrix<double, 3, 3> A;
    A << pow(tJerk, 3), pow(tJerk, 4), pow(tJerk, 5),
        3 * pow(tJerk, 2), 4 * pow(tJerk, 3), 5 * pow(tJerk, 4),
        6 * pow(tJerk, 1), 12 * pow(tJerk, 2), 20 * pow(tJerk, 3);

    Eigen::Matrix<double, 3, 3> P; // Min jerk parameters matrix
    Eigen::Matrix<double, 3, 1> V;
    double x_th, v_th, a_th, x_i;
    for (int i = 0; i < 3; i++)
    {
      x_th = TR.xThrow[i];
      v_th = TR.vThrow[i];
      a_th = TR.aThrow[i];
      x_i = pos_init[i];
      V[0] = x_th - x_i;
      V[1] = v_th;
      V[2] = a_th;
      P.col(i) = A.inverse() * V;
    }

    cout << "Throw position computed:\n"
         << TR.xThrow << "\nWith velocity:\n"
         << TR.vThrow << endl;

    // Orientating hand
    Quaternion quatThrow = Quaternion(1, 0, 0, 0);
    // quatThrow.X =  0.92388;
    // quatThrow.Y = 0.0;
    // quatThrow.Z = 0.38268;
    // quatThrow.W = 0.0;
    quatThrow = quatHand;

    hand_msg.data = 0.0;
    waitSec(3);
    t_init = ros::Time::now();
    t = (ros::Time::now() - t_init).toSec();
    while (t <= tEnd)
    {
      // cout << "Current time: " << t << endl;
      if (t >= (tThrow - time_adv) && fsHand)
      {
        pub_sh.publish(hand_msg);
        fsHand = false;
      }

      if (t < tThrow) // Minimum jerk phase (tThrow already scaled)
      {
        traj.pos_des.x() = pos_init.x() + P(0, 0) * pow(t / scale, 3) + P(1, 0) * pow(t / scale, 4) + P(2, 0) * pow(t / scale, 5);
        traj.pos_des.y() = pos_init.y() + P(0, 1) * pow(t / scale, 3) + P(1, 1) * pow(t / scale, 4) + P(2, 1) * pow(t / scale, 5);
        traj.pos_des.z() = pos_init.z() + P(0, 2) * pow(t / scale, 3) + P(1, 2) * pow(t / scale, 4) + P(2, 2) * pow(t / scale, 5);

        traj.vel_des.x() = (3 * P(0, 0) * pow(t / scale, 2) + 4 * P(1, 0) * pow(t / scale, 3) + 5 * P(2, 0) * pow(t / scale, 4)) / scale;
        traj.vel_des.y() = (3 * P(0, 1) * pow(t / scale, 2) + 4 * P(1, 1) * pow(t / scale, 3) + 5 * P(2, 1) * pow(t / scale, 4)) / scale;
        traj.vel_des.z() = (3 * P(0, 2) * pow(t / scale, 2) + 4 * P(1, 2) * pow(t / scale, 3) + 5 * P(2, 2) * pow(t / scale, 4)) / scale;

        traj.acc_des.x() = (6 * P(0, 0) * pow(t / scale, 1) + 12 * P(1, 0) * pow(t / scale, 2) + 20 * P(2, 0) * pow(t / scale, 3)) / pow(scale, 2);
        traj.acc_des.y() = (6 * P(0, 1) * pow(t / scale, 1) + 12 * P(1, 1) * pow(t / scale, 2) + 20 * P(2, 1) * pow(t / scale, 3)) / pow(scale, 2);
        traj.acc_des.z() = (6 * P(0, 2) * pow(t / scale, 1) + 12 * P(1, 2) * pow(t / scale, 2) + 20 * P(2, 2) * pow(t / scale, 3)) / pow(scale, 2);

        // Quaternion quatDes = Quaternion::Slerp(quatHand, quatThrow, t / tThrow);
        // traj.or_des = quatDes;
        traj.or_des = quatThrow;
      }

      else
      {
        // traj.acc_des.x() = 0;
        // traj.acc_des.y() = 0;
        // traj.acc_des.z() = -G;

        // Quaternion quatDes = Quaternion::Slerp(quatStart, quatThrow, 1.0);
        // traj.or_des = quatDes;
        traj.or_des = quatThrow;

        if (firstSwitch)
        {
          // pub_sh.publish(hand_msg);
          firstSwitch = false;

          throwValues.xThrow.x() = pos_init.x() + P(0, 0) * pow(t / scale, 3) + P(1, 0) * pow(t / scale, 4) + P(2, 0) * pow(t / scale, 5);
          throwValues.xThrow.y() = pos_init.y() + P(0, 1) * pow(t / scale, 3) + P(1, 1) * pow(t / scale, 4) + P(2, 1) * pow(t / scale, 5);
          throwValues.xThrow.z() = pos_init.z() + P(0, 2) * pow(t / scale, 3) + P(1, 2) * pow(t / scale, 4) + P(2, 2) * pow(t / scale, 5);

          throwValues.vThrow.x() = (3 * P(0, 0) * pow(t / scale, 2) + 4 * P(1, 0) * pow(t / scale, 3) + 5 * P(2, 0) * pow(t / scale, 4)) / scale;
          throwValues.vThrow.y() = (3 * P(0, 1) * pow(t / scale, 2) + 4 * P(1, 1) * pow(t / scale, 3) + 5 * P(2, 1) * pow(t / scale, 4)) / scale;
          throwValues.vThrow.z() = (3 * P(0, 2) * pow(t / scale, 2) + 4 * P(1, 2) * pow(t / scale, 3) + 5 * P(2, 2) * pow(t / scale, 4)) / scale;

          traj.pos_des.x() = throwValues.xThrow.x();
          traj.pos_des.y() = throwValues.xThrow.y();
          traj.pos_des.z() = throwValues.xThrow.z();

          traj.vel_des.x() = throwValues.vThrow.x();
          traj.vel_des.y() = throwValues.vThrow.y();
          traj.vel_des.z() = throwValues.vThrow.z();
          // cout << "Saved position:\n" << traj.pos_des << endl; Ok correct
          // t_init = ros::Time::now();
        }
        else
        {
          // traj.pos_des.x() = throwValues.xThrow.x() + throwValues.vThrow.x() * (t - tThrow);
          // traj.pos_des.y() = throwValues.xThrow.y() + throwValues.vThrow.y() * (t - tThrow);
          // traj.pos_des.z() = throwValues.xThrow.z() + throwValues.vThrow.z() * (t - tThrow) - 0.5 * G * pow((t - tThrow), 2);

          // traj.vel_des.x() = throwValues.vThrow.x();
          // traj.vel_des.y() = throwValues.vThrow.y();
          // traj.vel_des.z() = throwValues.vThrow.z() - G * (t - tThrow);
          interpolator_posSpeed(throwValues.xThrow, throwValues.vThrow, th, t - tThrow);
          cout << "Braking:\n"
               << traj.pos_des << endl;
          cout << "Braking time: " << t - tThrow << endl;
        }
      }

      // Now message has to be sent
      traj_msg.header.stamp = ros::Time::now();

      traj_msg.pose.position.x = traj.pos_des.x();
      traj_msg.pose.position.y = traj.pos_des.y();
      traj_msg.pose.position.z = traj.pos_des.z();

      traj_msg.velocity.position.x = traj.vel_des.x();
      traj_msg.velocity.position.y = traj.vel_des.y();
      traj_msg.velocity.position.z = traj.vel_des.z();

      traj_msg.acceleration.position.x = traj.acc_des.x();
      traj_msg.acceleration.position.y = traj.acc_des.y();
      traj_msg.acceleration.position.z = traj.acc_des.z();

      traj_msg.pose.orientation.x = traj.or_des.X;
      traj_msg.pose.orientation.y = traj.or_des.Y;
      traj_msg.pose.orientation.z = traj.or_des.Z;
      traj_msg.pose.orientation.w = traj.or_des.W;

      pub_cmd.publish(traj_msg);

      // Verifing limits
      // modV = sqrt(pow(traj.vel_des.x(), 2) + pow(traj.vel_des.y(), 2) + pow(traj.vel_des.z(), 2));
      // if (modV > MAX_V)
      // {
      //   cout << "Speed abs: " << modV << " --> saturating at time: " << t << endl;
      // }
      // else
      // {
      //   cout << "Speed abs: " << modV;
      // }

      // modA = sqrt(pow(traj.acc_des.x(), 2) + pow(traj.acc_des.y(), 2) + pow(traj.acc_des.z(), 2));
      // if (modA > MAX_A)
      // {
      //   cout << "Acceleration abs: " << modA << " --> saturating at time: " << t << endl;
      // }
      // // cout << "Trajectory message " << nM << " sent:\n" << traj_msg.pose.position << endl;
      // // cout << "Trajectory or message " << nM << " sent:\nx: " << traj_msg.pose.orientation.x << "\ny: " << traj_msg.pose.orientation.y << " \nz:" << traj_msg.pose.orientation.z << " \nw:" << traj_msg.pose.orientation.w << endl;
      // else
      // {
      //   cout << "  Acceleration abs: " << modA << endl;
      // }
      nM++;

      loop_rate.sleep();

      t = (ros::Time::now() - t_init).toSec();
      // cout << "init time: " << t_init << endl;
      // cout << "Current time: " << ros::Time::now() << endl;
      // cout << "End while time: " << t << endl;
    }
  }
}
