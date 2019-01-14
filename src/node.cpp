
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

Pose_ekf pose_ekf;  
ros::Publisher pub_vio_path;
ros::Publisher pub_vio_pose;
nav_msgs::Path path;


deque< pair<double, sensor_msgs::Imu> > imu_q;
deque< pair<double, geometry_msgs::PoseStamped> > slam_q;

void preintegration(const sensor_msgs::Imu::ConstPtr& msg)
{

   
    double t = msg->header.stamp.toSec();
    imu_q.push_back(make_pair(t, *msg));

    //part of publish_pose function is originally here
}

void slam(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

   
    double t = msg->header.stamp.toSec();
    slam_q.push_back(make_pair(t, *msg));

    //part of publish_pose function is originally here
}

bool processSensorData()
{
    if(imu_q.empty() || (imu_q.back().first - imu_q.front().first) < 0.15 ) //publish frequency???
        //cout<<" I am here "<<endl;
        return false;
    static int imu_cnt = 0;//correct with acc every 10 times

    double t[2] = {DBL_MAX};
    for(int i = 0; i < 2; i++) t[i] = DBL_MAX;
    if(!imu_q.empty()) t[0] = imu_q.front().first;
    if(!slam_q.empty()) t[1] = slam_q.front().first;  
    //for(int i = 0; i < 6; i++) cout << i << " " << t[i] << "  ";
    int min_id = min_element(t, t + 2) - t;
    //cout << "min_id: " << min_id << "  min_t: " << t[min_id] << endl;
    if(t[min_id] == DBL_MAX) return false;
    //...imu_q need data processing   return to pose_ekf

    if(min_id == 0)
    {
        double t = imu_q.front().first;
        sensor_msgs::Imu msg = imu_q.front().second;
        Vector3d acce, gyros;
        acce(0) = msg.linear_acceleration.x;
        acce(1) = msg.linear_acceleration.y;
        acce(2) = msg.linear_acceleration.z;
        gyros(0) = msg.angular_velocity.x;
        gyros(1) = msg.angular_velocity.y;
        gyros(2) = msg.angular_velocity.z;
        pose_ekf.predict(gyros, acce, t);
        imu_cnt++;
        if(imu_cnt % 10 == 0) 
            pose_ekf.correct_gravity(acce);//t);
        imu_q.pop_front();
    }
   
    if(min_id == 1)
    {
        //measurement update  
        double tt = slam_q.front().first;
        geometry_msgs::PoseStamped smsg = slam_q.front().second;  //modify here begin!!!
        Vector3d position;
        position(0)=smsg.pose.position.x;
        position(1)=smsg.pose.position.y;
        position(2)=smsg.pose.position.z;
        Quaterniond sq;
        sq.w()=smsg.pose.orientation.w;
        sq.x()=smsg.pose.orientation.x;
        sq.y()=smsg.pose.orientation.y;
        sq.z()=smsg.pose.orientation.z;
        pose_ekf.correct_slam(position, sq, tt);
        slam_q.pop_front();
    }

    return true;

}

void publish_pose(Pose_ekf& pose_ekf)
{
        //publish pose and path
        geometry_msgs::PoseStamped pose;
        Quaterniond q;
        Vector3d p, v, bw, ba;
        pose_ekf.getState(q, p, v, bw, ba);  //get ekf state
        pose.header.stamp = ros::Time(pose_ekf.get_time());
        pose.header.frame_id = "map";
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
    ros::Rate r(50);

    while(ros::ok())
    {
        ros::spinOnce();
        //cout<<imu_q.empty()<<endl;
        while(processSensorData()){
            //cout<<"IN"<<endl;
        }
        //cout<<imu_q.empty()<<endl;
        //cout<<"OUT"<<endl;
        publish_pose(pose_ekf); //ekf class

        r.sleep();
    }
    return 0;
}