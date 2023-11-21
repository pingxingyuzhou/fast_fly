#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

using namespace std;
#define pi 57.2957805
typedef struct Quaternion {
    double w;
    double x;
    double y;
    double z;
} Quaternion;

typedef struct EulerAngle {
    double roll;
    double pitch;
    double yaw;
} EulerAngle;

int uwb_type=0;//0 nlink,1 wit;
float yaw_err;
int yaw_turn=0;//0,zuo - you +
EulerAngle imu_angle;
Quaternion imu_q;
nav_msgs::Odometry odometry;
ros::Publisher m_pub;

Quaternion ToQuaternion(double yaw, double pitch, double roll);
EulerAngle ToEulerAngles(Quaternion q);
float q_t265_w=0;
float q_t265_x=0;
float q_t265_y=0;
float q_t265_z=0;





	void wit_uwb_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
   
		odometry.header.frame_id = "uwb_odom_frame";
		odometry.pose.pose.position.x = msg->pose.pose.position.x;
		odometry.pose.pose.position.y = msg->pose.pose.position.y;
		odometry.pose.pose.position.z = msg->pose.pose.position.z;
		
}
	void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
   
    
        EulerAngle t265_angle;
		Quaternion t265_q;
		t265_q.w=msg->pose.pose.orientation.w;
		t265_q.x=msg->pose.pose.orientation.x;
		t265_q.y=msg->pose.pose.orientation.y;
		t265_q.z=msg->pose.pose.orientation.z;
		t265_angle=ToEulerAngles(t265_q);
		// std::cout<<"t265_angle.roll="<<t265_angle.roll*pi<<std::endl;
		// std::cout<<"t265_angle.pitch="<<t265_angle.pitch*pi<<std::endl;
		// std::cout<<"t265_angle.yaw="<<t265_angle.yaw*pi<<std::endl;

		
       
}
	
	void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
		EulerAngle test_angle;
		Quaternion test_q;
		double yaw=0.0;
	
        
		test_q.w=msg->orientation.w;
		test_q.x=msg->orientation.x;
		test_q.y=msg->orientation.y;
		test_q.z=msg->orientation.z;
		test_angle=ToEulerAngles(test_q);
		yaw=test_angle.yaw*pi;
		std::cout<<"test_angle.roll="<<test_angle.roll*pi<<std::endl;
		std::cout<<"test_angle.pitch="<<test_angle.pitch*pi<<std::endl;
		std::cout<<"test_angle.yaw="<<test_angle.yaw*pi<<std::endl;
		std::cout<<"yaw_err="<<yaw_err<<std::endl;

		if(yaw_err>0)
	{
		yaw=yaw-yaw_err;
		if(yaw>=180)
		yaw=yaw-360;
		else if(yaw<-180)
		yaw=yaw+360;
	}else if(yaw_err<0)
	
		{
		yaw=yaw-yaw_err;
		if(yaw<-180)
		yaw=yaw+360;
		else if(yaw>=180)
		yaw=yaw-360;
	}
	// if(yaw_turn==0)
	// yaw=-yaw;
	

	//////////////////////////////////////////////////////////////////////
	

		std::cout<<"yaw="<<yaw<<std::endl;
		                             
		imu_q=ToQuaternion(yaw, test_angle.pitch, test_angle.roll);
		odometry.pose.pose.orientation.w=imu_q.w;
		odometry.pose.pose.orientation.x=imu_q.x;
		odometry.pose.pose.orientation.y=imu_q.y;
		odometry.pose.pose.orientation.z=imu_q.z;
		
		m_pub.publish(odometry);
		
       
}
void vision_to_mavros_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   
    
        EulerAngle vision_angle;
		Quaternion vision_q;
		vision_q.w=msg->pose.orientation.w;
		vision_q.x=msg->pose.orientation.x;
		vision_q.y=msg->pose.orientation.y;
		vision_q.z=msg->pose.orientation.z;
		vision_angle=ToEulerAngles(vision_q);
		// std::cout<<"vision_angle.roll="<<vision_angle.roll*pi<<std::endl;
		// std::cout<<"vision_angle.pitch="<<vision_angle.pitch*pi<<std::endl;
		// std::cout<<"vision_angle.yaw="<<vision_angle.yaw*pi<<std::endl;
       
}

void fcu_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
   
    
        EulerAngle fcu_angle;
		Quaternion fcu_q;
		fcu_q.w=msg->pose.pose.orientation.w;
		fcu_q.x=msg->pose.pose.orientation.x;
		fcu_q.y=msg->pose.pose.orientation.y;
		fcu_q.z=msg->pose.pose.orientation.z;
		fcu_angle=ToEulerAngles(fcu_q);

		std::cout<<"fcu_angle.roll="<<fcu_angle.roll*pi<<std::endl;
		std::cout<<"fcu_agle.pitch="<<fcu_angle.pitch*pi<<std::endl;
		std::cout<<"fcu_angle.yaw="<<fcu_angle.yaw*pi<<std::endl;
       
}

int main(int argc, char **argv)
{
		string  t265_topic;
		string imu_topic;
		string fcu_topic;
		string vision_topic;
		string uwb_topic_pub;

		ros::Subscriber m_sub;
		ros::Subscriber fcu_sub;
		ros::Subscriber vision_to_mavros_sub;
		
		ros::Subscriber t265_sub;
		ros::Subscriber imu_sub;

	    ros::init(argc, argv, "poseTransfer_node");
		ros::NodeHandle nh("~");
	   nh.param<string>("t265_topic",t265_topic,"/camera/odom/sample") ;
		nh.param<string>("imu_topic",imu_topic,"/wit/imu");
		nh.param<string>("vision_topic",vision_topic,"/mavros/vision_pose/pose");
		nh.param<string>("uwb_topic_pub",uwb_topic_pub,"uwb_pose/pose");
		nh.param<string>("fcu_topic",fcu_topic,"/mavros/local_position/odom");
		nh.param<float>("yaw_err", yaw_err, 0);
		nh.param<int>("yaw_turn", yaw_turn, 0);
		nh.param<int>("uwb_type", uwb_type, 0);
		//yaw_err=yaw_err-90;
		
		
		m_sub=nh.subscribe<nav_msgs::Odometry>("/wit_uwb_pose_xy/pose",10, wit_uwb_cb);
		t265_sub = nh.subscribe<nav_msgs::Odometry>(t265_topic, 10,t265_cb);
		fcu_sub = nh.subscribe<nav_msgs::Odometry>(fcu_topic, 10, fcu_cb);
		imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic,10, imu_cb);
		vision_to_mavros_sub = nh.subscribe<geometry_msgs::PoseStamped>(vision_topic,100, vision_to_mavros_cb);
		m_pub = nh.advertise<nav_msgs::Odometry>(uwb_topic_pub, 1000);
	
	
	
	 //PoseTransfer poseTransfer;

	 ros::spin();
	 

	return 0;
	
}

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
 
    return q;
}
 

EulerAngle ToEulerAngles(Quaternion q) {
    EulerAngle angles;
 
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
 
    return angles;
}

