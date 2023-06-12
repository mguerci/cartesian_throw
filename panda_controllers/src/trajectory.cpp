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
#include "utils/parsing_utilities.h"
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

// struct angleData
// {
//   double theta;
//   double v;
//   bool found;
// };

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
int PP_time = 5; // Time for pick & place task
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

throwData computeThrow(Eigen::Vector3d xStart, Eigen::Vector3d xGround, double marg)
{
  double alpha, beta, gamma, r, c, t_2, start, s, dx, V_new, eta;
  double V = 100000000, R = 0.855, step = 0.0001;
  double temp;
  R = R - marg;
  Eigen::Matrix<double, 3, 1> u;
  u.setZero();
  u(2) = 1;
  Eigen::Matrix<double, 2, 1> P_1;
  Eigen::Matrix<double, 3, 1> v;
  v(0) = xGround.x() - xStart.x();
  v(1) = xGround.y() - xStart.y();
  v(2) = 0;
  v = (1 / v.norm()) * v;

  Eigen::Vector3d xThrow_new;
  xThrow_new.setZero();

  alpha = 2 * (xStart.x() * v(0) + xStart.y() * v(1));
  beta = 2 * xStart.z();
  gamma = -(pow(R, 2) - pow(xStart.x(), 2) - pow(xStart.y(), 2) - pow(xStart.z(), 2));
  r = sqrt(0.25 * pow(alpha, 2) + 0.25 * pow(beta, 2) - gamma);
  c = pow(r, 2) - 0.25 * pow(beta, 2);

  t_2 = -0.25 * beta + 0.5 * sqrt(pow(beta, 2) + 4 * c);
  start = -xStart.z();

  Eigen::Vector3d xThrow;
  xThrow.setZero();
  Eigen::Vector3d vThrow;
  vThrow.setZero();
  Eigen::Vector3d aThrow;
  aThrow.setZero();
  throwData result;

  for (double t = start; t <= t_2; t += step)
  {
    temp = pow(r, 2) - pow((t + 0.5 * beta), 2);
    if (temp < 0)
      continue;

    s = -0.5 * alpha + sqrt(temp);

    xThrow_new.x() = xStart.x() + s * v(0);
    xThrow_new.y() = xStart.y() + s * v(1);
    xThrow_new.z() = xStart.z() + t * u(2);

    if (xThrow_new.z() < 0.4)
      continue;

    dx = sqrt(pow((xGround.x() - xThrow_new.x()), 2) + pow((xGround.y() - xThrow_new.y()), 2));
    P_1(0) = dx;
    P_1(1) = xGround.z();

    temp = (G * pow(P_1(0), 2)) / (P_1(0) - P_1(1) + xThrow_new.z());
    if (temp < 0)
      continue;
    V_new = sqrt(temp);
    if (V_new < V)
    {
      result.found = true;
      V = V_new;
      xThrow = xThrow_new;
    }
  }

  if (result.found)
  {
    eta = atan2(xGround.y() - xStart.y(), xGround.x() - xStart.x());
    vThrow.x() = V * cos(eta) * cos(M_PI_4);
    vThrow.y() = V * sin(eta) * cos(M_PI_4);
    vThrow.z() = V * sin(M_PI_4);
  }

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

void move_EE(Eigen::Vector3d posEnd, Quaternion curr_or, ros::Publisher pub_cmd)
{
  ros::Time t_init;
  double t, tr;
  ros::Rate loop_rate(RATE_FREQ);
  panda_controllers::DesiredTrajectory traj_msg;
  Eigen::Vector3d posStart;
  t_init = ros::Time::now();
  waitForPos();
  posStart = pos;
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

void rotate_EE(Quaternion quatEnd, ros::Publisher pub_cmd)
{
  ros::Time t_init;
  double t;
  ros::Rate loop_rate(RATE_FREQ);
  panda_controllers::DesiredTrajectory traj_msg;
  Quaternion quatStart;
  Eigen::Vector3d curr_pos;
  waitForPos();
  quatStart.X = orient.x;
  quatStart.Y = orient.y;
  quatStart.Z = orient.z;
  quatStart.W = orient.w;
  curr_pos = pos;
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

void moveRotate_EE(Quaternion quatEnd, Eigen::Vector3d posEnd, ros::Publisher pub_cmd)
{
  ros::Time t_init;
  double t, tr;
  ros::Rate loop_rate(RATE_FREQ);
  panda_controllers::DesiredTrajectory traj_msg;
  Quaternion quatStart;
  Eigen::Vector3d posStart;
  waitForPos();
  quatStart.X = orient.x;
  quatStart.Y = orient.y;
  quatStart.Z = orient.z;
  quatStart.W = orient.w;
  posStart = pos;
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
  ros::Publisher pub_real_sh = node_handle.advertise<std_msgs::Float64>("/qb_class/hand_ref", 1);

  ros::Subscriber sub_cmd = node_handle.subscribe("/project_impedance_controller/franka_ee_pose", 1,
                                                  &poseCallback);

  ros::Rate loop_rate(100);

  signal(SIGINT, signal_callback_handler);

  XmlRpc::XmlRpcValue throw_par;
  Eigen::MatrixXd parser;
  if (!node_handle.getParam("/throw_par", throw_par))
  {
    ROS_ERROR("Could not get the XmlRpc value.");
  }
  panda_controllers::DesiredTrajectory traj_msg;

  Eigen::Vector3d pos_f;
  Eigen::Vector3d pos_init;
  Eigen::Vector3d or_init;
  Eigen::Vector3d obj_init;
  Eigen::Vector3d pos_bar;
  Eigen::Vector3d pos_target;
  Eigen::Vector3d pos_startThrow;

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
  std_msgs::Float64 real_hand_msg;

  if (!parseParameter(throw_par, PP_time, "PP_time"))
  {
    ROS_ERROR("Could not parse traj_par PP_time.");
  }
  cout << "Setting PP_time: " << PP_time << endl;
  n = 2;
  m = 110;
  mat = getMatrix("/home/matteo/catkin_ws/src/panda_controllers/Matrix.txt", n, m);
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
  pos_init = pos;
  Quaternion quatStart = Quaternion(orient.x, orient.y, orient.z, orient.w);

  // New: with real time
  double ratio;
  int nM = 0;

  if (demo)
  {
    waitForPos();
    move_EE(pos_bar, quatStart, pub_cmd);
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
    real_hand_msg.data = 0.0;
    pub_real_sh.publish(real_hand_msg);
    waitSec(3);
    cout << "Hand full open" << endl;

    Quaternion quatHand;
    quatHand.X = -0.6533;
    quatHand.Y = -0.2706;
    quatHand.Z = -0.2706;
    quatHand.W = 0.6533;

    // SIMULATION
    // quatHand.X = 0.7;
    // quatHand.Y = 0.0356;
    // quatHand.Z = 0.7144;
    // quatHand.W = 0.0086;
    quatHand.X = -0.72;
    quatHand.Y = -0.0188;
    quatHand.Z = -0.69288;
    quatHand.W = 0.02359;

    if (!parseParameter(throw_par, parser, "quatGrasp"))
    {
      ROS_ERROR("Could not parse traj_par quatGrasp.");
    }
    quatHand.X = parser(0, 0);
    quatHand.Y = parser(0, 1);
    quatHand.Z = parser(0, 2);
    quatHand.W = parser(0, 3);
    quatHand = normalizeQuat(quatHand);
    // cout << "Orientation: " << quatHand.X << " "<< quatHand.Y << " "<< quatHand.Z << " "<< quatHand.W << endl;
    // quatHand.X = 0.26984;
    // quatHand.Y = -0.643745;
    // quatHand.Z = 0.649614;
    // quatHand.W = 0.30124;

    // waitForPos();
    // move_EE(pos_bar, quatStart, pub_cmd);
    // rotate_EE(quatHand, pub_cmd);

    // obj_init.x() = 0.38;
    // obj_init.y() = -0.26;
    // obj_init.z() = 0.02;

    // obj_init.x() = 0.38;
    // obj_init.y() = -0.26;
    // obj_init.z() = 0.06;

    // REAL VALUES
    // obj_init.x() = 0.55;
    // obj_init.y() = -0.35;
    // obj_init.z() = 0.05;

    // SIMULATION VALUES
    obj_init.x() = 0.538;
    obj_init.y() = -0.25;
    obj_init.z() = 0.186;
    if (!parseParameter(throw_par, parser, "obj_position"))
    {
      ROS_ERROR("Could not parse traj_par obj_position.");
    }
    obj_init.x() = parser(0, 0);
    obj_init.y() = parser(0, 1);
    obj_init.z() = parser(0, 2);

    pos_target = obj_init;
    pos_target.z() = obj_init.z() + 0.25;
    moveRotate_EE(quatHand, pos_target, pub_cmd);
    // waitForPos();
    // move_EE(pos_target, quatHand, pub_cmd);
    waitForPos();
    move_EE(obj_init, quatHand, pub_cmd);
    cout << "Here comes the hand" << endl;
    // cout << "Waiting for enter to close" << endl;
    // while (cin.get() != '\n');
    hand_msg.data = 0.9;
    pub_sh.publish(hand_msg);
    real_hand_msg.data = 10000;
    pub_real_sh.publish(real_hand_msg);
    cout << "Hand closing" << endl;
    waitSec(3);
    cout << "Hand closed" << endl;

    // cout << "Going to q_bar position" << endl;
    // waitForPos();
    // move_EE(pos_bar, quatHand, pub_cmd);

    // pos_startThrow.x() = 0.48;
    // pos_startThrow.y() = -0.4;
    // pos_startThrow.z() = 0.46;
    pos_startThrow.x() = 0.4;
    pos_startThrow.y() = -0.3;
    pos_startThrow.z() = 0.5;
    if (!parseParameter(throw_par, parser, "pos_startThrow"))
    {
      ROS_ERROR("Could not parse traj_par pos_startThrow.");
    }
    pos_startThrow.x() = parser(0, 0);
    pos_startThrow.y() = parser(0, 1);
    pos_startThrow.z() = parser(0, 2);
    cout << "Going to start throw position" << endl;
    waitForPos();
    // move_EE(pos_startThrow, quatHand, pub_cmd);
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
    // quatHand.X = 0.6284;
    // quatHand.Y = -0.5777;
    // quatHand.Z = 0.38434;
    // quatHand.W = -0.351634;

    quatHand.X = 0.840174;
    quatHand.Y = -0.330113;
    quatHand.Z = 0.192424;
    quatHand.W = -0.384847;
    if (!parseParameter(throw_par, parser, "quatThrow"))
    {
      ROS_ERROR("Could not parse traj_par quatThrow.");
    }
    quatHand.X = parser(0, 0);
    quatHand.Y = parser(0, 1);
    quatHand.Z = parser(0, 2);
    quatHand.W = parser(0, 3);
    quatHand = normalizeQuat(quatHand);
    moveRotate_EE(quatHand, pos_startThrow, pub_cmd);

    // Rotating hand in throw orientation
    quatStart = quatHand;
    // cout << "Select throw mode: ";
    // cin >> thMode;
    thMode = 4;
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
      marg = 0.15;
      // Quaternione per lancio dritto su x
      // quatHand.X = -0.262875;
      // quatHand.Y = 0.398214;
      // quatHand.Z = -0.426742;
      // quatHand.W = 0.768254;

      // // Quaternione per lancio obliquo
      // quatHand.X = -0.2484;
      // quatHand.Y = 0.4319;
      // quatHand.Z = 0.2944;
      // quatHand.W = 0.8169;

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
      // For y-throw
      marg = 0.15;

      // Throw
      pos_f.x() = 1;
      pos_f.y() = 0.3;
      pos_f.z() = 0;

      scale = 1;
      tJerk = 1;
      time_adv = 0.25;
    }

    // rotate_EE(quatHand, pub_cmd);

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

    // TR = computeThrow(pos_f, mat, n, m, marg);
    TR = computeThrow(pos_startThrow, pos_f, marg);
    if (!TR.found)
    {
      ROS_ERROR_STREAM("Trajectory: Could not compute throw parameters ");
      return false;
    }

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
    Quaternion quatThrow;
    quatThrow = quatHand;

    hand_msg.data = 0.0;
    real_hand_msg.data = 0.0;
    waitSec(3);
    t_init = ros::Time::now();
    t = (ros::Time::now() - t_init).toSec();
    while (t <= tEnd)
    {
      // cout << "Current time: " << t << endl;
      if (t >= (tThrow - time_adv) && fsHand)
      {
        pub_sh.publish(hand_msg);
        pub_real_sh.publish(real_hand_msg);
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
          // cout << "Braking:\n"
          //      << traj.pos_des << endl;
          // cout << "Braking time: " << t - tThrow << endl;
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
