#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

mavros_msgs::State current_state;
double g_x = 0;
double g_y = 0;
double g_z = 0;

double g_beginX = 0;
double g_beginY = 0;
double g_beginZ = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){

	current_state = *msg;
	//printf("=====:arm:%d, connect:%d, guided:%d, mode:%s, frame_id:%s.\n", current_state.armed, current_state.connected, current_state.guided, current_state.mode.c_str(), current_state.header.frame_id.c_str());
}


bool g_bGetPose = false;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	ros::Time timestamp = msg->header.stamp;
	uint32_t seq = msg->header.seq;
	std::string frame_id = msg->header.frame_id;
	g_x = msg->pose.position.x;
	g_y = msg->pose.position.y;
	g_z = msg->pose.position.z;
	//printf("-------x:%f.\n", g_x);
	
	g_bGetPose = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offboard_node");
	ros::NodeHandle nh;
	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	

	//PoseTransfer iPoseTransfer;

	ros::Rate rate(50.0);
	
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	while(ros::ok() && !g_bGetPose){
		printf("not yet get pose cb.\n");
		ros::spinOnce();
		rate.sleep();
	}
	
	g_beginX = g_x;
	g_beginY = g_y;
	g_beginZ = g_z;

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = g_x;
	pose.pose.position.y = g_y;
	pose.pose.position.z = g_z + 0.43;

	printf("target offboard fly is : (%f, %f, %f).\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);


	for(int i= 0; ros::ok() && i < 100; ++i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	
	ros::Time last_request = ros::Time::now();

	std::vector<geometry_msgs::Point> vPoints;

	int i = 0;
	while(ros::ok()){
		if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
			if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else 
        {
			if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
				if(arming_client.call(arm_cmd) && arm_cmd.response.success){
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}


		//run, 16-60s
		if(i > 1000){
			pose.pose.position.x += 0.01;
			if(pose.pose.position.x > 2.2){
				pose.pose.position.x = 2.2;
			}
		}
		/*if(i > 3000){
			pose.pose.position.y += 0.01;
			if(pose.pose.position.y > 2){
				pose.pose.position.y = 2;
			}
		}
		if(i > 4500){
			pose.pose.position.x -= 0.01;
			if(pose.pose.position.x < 0.5){
				pose.pose.position.x = 0.5;
			}
		}
		if(i > 7500){
			pose.pose.position.x = g_beginX;
			pose.pose.position.y = g_beginY;
		}*/
		// if(i> 1500){
		// 	pose.pose.position.z -= 0.01;
		// 	if(pose.pose.position.z < 0.15){
		// 		break;
		// 	}
		// }
		
		pose.header.stamp = ros::Time::now();
		local_pos_pub.publish(pose);

		i++;
		ros::spinOnce();
		rate.sleep();
	}
	//land
	offb_set_mode.request.custom_mode = "AUTO.LAND";
	if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
		ROS_INFO("AUTO.LAND enabled");
	}
	
	return 0;
	
}




