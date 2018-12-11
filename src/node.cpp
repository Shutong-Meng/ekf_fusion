
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <Eigen/Geometry>
#include <deque>
#include "pose_ekf.h"
#include <algorithm>

using namespace std;
using namespace Eigen;

Pose_ekf pose_ekf;  //?????
ros::Publisher pub_vio_path;
ros::Publisher pub_vio_pose;
nav_msgs::Path path;

static int imu_cnt = 0;
deque< pair<double, sensor_msgs::Imu> > imu_q;
deque< pair<double, geometry_msgs::PointStamped> > slam_q;

void preintegration(const sensor_msgs::Imu::ConstPtr& msg)
{

   
    double t = msg->header.stamp.toSec();
    imu_q.push_back(make_pair(t, *msg));

    //part of publish_pose function is originally here
}

void slam(const geometry_msgs::PointStamped::ConstPtr& msg)
{

   
    double t = msg->header.stamp.toSec();
    slam_q.push_back(make_pair(t, *msg));

    //part of publish_pose function is originally here
}

void publish_pose(Pose_ekf pose_ekf)
{

    //...imu_q need data processing   return to pose_ekf
    double t = imu_q.front().first;
    sensor_msgs::Imu msg = imu_q.front().second;
    Vector3d acc, gyro;
    acc(0) = msg.linear_acceleration.x;
    acc(1) = msg.linear_acceleration.y;
    acc(2) = msg.linear_acceleration.z;
    gyro(0) = msg.angular_velocity.x;
    gyro(1) = msg.angular_velocity.y;
    gyro(2) = msg.angular_velocity.z;
    pose_ekf.predict(gyro, acc, t);
    imu_cnt++;
    if(imu_cnt % 10 == 0) 
        pose_ekf.correct_gravity(acc, t);
    imu_q.pop_front();

    //measurement update  xuyaotianjia q bufen  xianzaizhiyouweizhi
    double tt = slam_q.front().first;
    geometry_msgs::PointStamped smsg = slam_q.front().second;
    Vector3d position;
    position(0)=smsg.point.x;
    position(1)=smsg.point.y;
    position(2)=smsg.point.z;
    pose_ekf.correct_slam(position, tt);
    slam_q.pop_front();

    //publish pose and path
    geometry_msgs::PoseStamped pose;
    Quaterniond q;
    Vector3d p, v, bw, ba;
    pose_ekf.getState(q, p, v, bw, ba);  //get ekf state
    pose.header.stamp = ros::Time(pose_ekf.get_time());
    pose.header.frame_id = "world";
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);
    pub_vio_pose.publish(pose);

    path.header.frame_id ="map";
    path.poses.push_back(pose);
    pub_vio_path.publish(path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_pose");
    ros::NodeHandle n;
    pub_vio_path = n.advertise<nav_msgs::Path>("/ekf/path_viz", 1000);
    pub_vio_pose = n.advertise<geometry_msgs::PoseStamped>("/ekf/pose", 1000);
    //ros::Rate r(10);  //hz
    ros::Subscriber sub_imu = n.subscribe("/imu0", 1000, preintegration);
    ros::Subscriber sub_slam = n.subscribe("/Stereo/position", 1000, slam);
    ros::Rate r(20);

    while(ros::ok())
    {
    ros::spinOnce();
    //cout<<imu_q.empty()<<endl;
    if(!imu_q.empty()&&!slam_q.empty())
   
    //cout<<imu_q.empty()<<endl;
    publish_pose(pose_ekf); //ekf class

    r.sleep();
    }
    return 0;
}