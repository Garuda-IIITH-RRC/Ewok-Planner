/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
// #include <sstream>

#include "bspline_lee_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

// ros::Publisher wp_pub;


namespace rotors_control {



BSplineLeePositionControllerNode::BSplineLeePositionControllerNode(
        ros::NodeHandle & nh): nh(nh) {

  ROS_INFO("Started BSplineLeePositionControllerNode constructor");

  // InitializeParams();


  cmd_poly_sub_ = nh.subscribe(
      "command/point", 10,
      &BSplineLeePositionControllerNode::PointCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &BSplineLeePositionControllerNode::OdometryCallback, this);

  wp_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",30);


    /* Services */
  // arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // initPose.pose.position.x = 0;
  // initPose.pose.position.y = 0;
  // initPose.pose.position.z = 2.5;

  // ros::Rate rate(20);
  
  // for(int i = 0; i<20 && ros::ok(); i++)
  //   {
  //     wp_pub.publish(initPose);
  //     ros::spinOnce();
  //     rate.sleep();
  //   }

  //   mavros_msgs::SetMode offb_set_mode;
  //   offb_set_mode.request.custom_mode = "OFFBOARD";

  //   mavros_msgs::CommandBool arm_cmd;
  //   arm_cmd.request.value = true;

  //       /** set mode to offboard **/
  //   if(set_mode_client.call(offb_set_mode) &&
  //       offb_set_mode.response.mode_sent)
  //     {
  //       ROS_INFO("Offboard enabled");
  //     }

  // wp_pub.publish(pos);

  ros::NodeHandle pnh("~");
  pnh.param("dt", dt, 0.5);

  b_spline_.reset(new ewok::UniformBSpline3D<6, double>(dt));


  ROS_INFO("Finished BSplineLeePositionControllerNode constructor");

}

BSplineLeePositionControllerNode::~BSplineLeePositionControllerNode() { }


void BSplineLeePositionControllerNode::PointCallback(const geometry_msgs::PointConstPtr & point_msg) {

  // ROS_INFO("PointCallback: %d points in spline", b_spline_->size());

  Eigen::Vector3d p(point_msg->x, point_msg->y, point_msg->z);

  // ROS_INFO_STREAM("p: " << p);
  // position(0) = point_msg->x;
  // position(1) = point_msg->y;
  // position(2) = point_msg->z;

  if(b_spline_->size() != 0) b_spline_->push_back(p);


}

void BSplineLeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  // nav_msgs::Odometry odom;

  // odom.pose.pose.position.x = odometry_msg->pose.pose.position.x;
  // odom.pose.pose.position.z = odometry_msg->pose.pose.position.z;


  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);



  if(b_spline_->size() == 0) {
    for(int i=0; i<6; i++) b_spline_->push_back(odometry.position);
    init_time = ros::Time::now();
    last_yaw = mav_msgs::yawFromQuaternion(odometry.orientation);
  }

  double local_t = (ros::Time::now() - init_time).toSec();

  if(local_t > b_spline_->maxValidTime()) {
    b_spline_->push_back(b_spline_->getControlPoint(b_spline_->size()-1));
    ROS_WARN("Adding last point once again!");
  }

  bool yaw_from_traj;
  mav_msgs::EigenTrajectoryPoint command_trajectory;
  getTrajectoryPoint(local_t, command_trajectory, yaw_from_traj);

  if(!yaw_from_traj) {
    command_trajectory.setFromYaw(last_yaw);
    command_trajectory.setFromYawRate(0);
  } else {
    last_yaw = command_trajectory.getYaw();
  }


  SetTrajectoryPoint(command_trajectory);
  // SetOdometry(odometry);


  if (!controller_active_) {
    pos.header.stamp = ros::Time::now();
    pos.pose.position.x = initPose.pose.position.x;
    pos.pose.position.y = initPose.pose.position.y;
    pos.pose.position.z = initPose.pose.position.z;
    // ROS_INFO_STREAM("pos_callback: " << pos);
    wp_pub.publish(pos);
  }

  else{
  Eigen::Vector3d position;
  position(0) = command_trajectory.position_W(0);
  position(1) = command_trajectory.position_W(1);
  position(2) = command_trajectory.position_W(2);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY( 0, 0, command_trajectory.getYaw());
  myQuaternion=myQuaternion.normalize();

  pos.header.stamp = ros::Time::now();
  pos.pose.position.x = position(0);
  pos.pose.position.y = position(1);
  pos.pose.position.z = position(2);
  pos.pose.orientation.x = myQuaternion[0];
  pos.pose.orientation.y = myQuaternion[1];
  pos.pose.orientation.z = myQuaternion[2];
  pos.pose.orientation.w = myQuaternion[3];
  // ROS_INFO_STREAM("pos_callback: " << pos);
  wp_pub.publish(pos);

  }

  // Eigen::Vector4d velocity;
  // velocity(0) = command_trajectory.velocity_W(0);
  // velocity(1) = command_trajectory.velocity_W(1);
  // velocity(2) = command_trajectory.velocity_W(2);
  // velocity(4) = command_trajectory.getYawRate();

  // vel_cmd.linear.x = velocity(0);
  // vel_cmd.linear.y = velocity(1);
  // vel_cmd.linear.z = velocity(2);
  // vel_cmd.angular.z = velocity(4);

  // ROS_INFO_STREAM("pos_callback: " << vel_cmd);
  // vel_pub.publish(vel_cmd);


  // ROS_INFO_STREAM("odometry: " << odom->odometry_msg.pose.position.x);
  // ROS_INFO_STREAM("odometry: " << odom->odometry_msg.pose.position.x);
  // ROS_INFO_STREAM("pos: " << pos);
  // ROS_INFO_STREAM("pos: " << pos);
  // mavros_msgs::SetMode offb_set_mode;
  // offb_set_mode.request.custom_mode = "OFFBOARD";

  // mavros_msgs::CommandBool arm_cmd;
  // arm_cmd.request.value = true;
  // ros::Publisher motor_velocity_reference_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/setpoint_point",10);
  


// #Publish using pointers

  // geometry_msgs::PoseStampedPtr wp_msg(new geometry_msgs::PoseStamped);
  // // // wp_msg->angular_velocities.clear();
  // // // wp_msg->Pose.clear();
  // // // wp_msg->pose.position.y.clear();
  // // // wp_msg->pose.position.z.clear();
  // // // wp_msg->pose.orientation.x.clear();
  // // // wp_msg->pose.orientation.y.clear();
  // // // wp_msg->pose.orientation.z.clear();
  // // // wp_msg->pose.orientation.w.clear();

  // wp_msg->header.stamp = odometry_msg->header.stamp;
  // wp_msg->pose.position.x = position(0);
  // wp_msg->pose.position.y = position(1);
  // wp_msg->pose.position.z = position(2);
  // wp_msg->pose.orientation.x = myQuaternion[0];
  // wp_msg->pose.orientation.y = myQuaternion[1];
  // wp_msg->pose.orientation.z = myQuaternion[2];
  // wp_msg->pose.orientation.w = myQuaternion[3];
  // ROS_INFO_STREAM("wp_msg: " << wp_msg);
  // // wp_pub.publish(wp_msg);


  // lee_position_controller_.SetTrajectoryPoint(command_trajectory);

  // ROS_INFO_STREAM("Setting point: " << command_trajectory.position_W.transpose());
  // ROS_INFO_STREAM("Setting point: " << command_trajectory.getYaw());

  // lee_position_controller_.SetOdometry(odometry);

  // Eigen::VectorXd ref_rotor_velocities;
  // lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // // Todo(ffurrer): Do this in the conversions header.
  // mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  // actuator_msg->angular_velocities.clear();
  // for (int i = 0; i < ref_rotor_velocities.size(); i++)
  //   actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  // actuator_msg->header.stamp = odometry_msg->header.stamp;

  // motor_velocity_reference_pub_.publish(actuator_msg);
}



void BSplineLeePositionControllerNode::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void BSplineLeePositionControllerNode::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void BSplineLeePositionControllerNode::getTrajectoryPoint(double t,
      mav_msgs::EigenTrajectoryPoint& command_trajectory, bool & yaw_from_traj) {


  command_trajectory.position_W = b_spline_->evaluate(t, 0);
  command_trajectory.velocity_W = b_spline_->evaluate(t, 1);
  command_trajectory.acceleration_W = b_spline_->evaluate(t, 2);

  static const double eps = 0.1;
  static const double delta = 0.02;

  Eigen::Vector3d d_t = b_spline_->evaluate(t + eps, 0) - command_trajectory.position_W;

  yaw_from_traj = false;

  if(std::abs(d_t[0]) > delta || std::abs(d_t[1]) > delta) {
    double yaw = std::atan2(d_t[1], d_t[0]);
    yaw_from_traj = true;

    command_trajectory.setFromYaw(yaw);


    Eigen::Vector3d d_t_e = b_spline_->evaluate(t + 2*eps, 0) - b_spline_->evaluate(t + eps, 0);

    if(std::abs(d_t_e[0]) > delta || std::abs(d_t_e[1]) > delta) {
      double yaw_e = std::atan2(d_t_e[1], d_t_e[0]);
      double yaw_rate = (yaw_e - yaw) / eps;
      command_trajectory.setFromYawRate(yaw_rate);
    } else {
      command_trajectory.setFromYawRate(0);
    }

  }

}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node2");
  ros::NodeHandle nh;

  // geometry_msgs::PoseStamped pos;

  rotors_control::BSplineLeePositionControllerNode lee_position_controller_node(nh);


  ros::spin();

  return 0;
}
