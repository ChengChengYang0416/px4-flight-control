/**
 * @file offb_main.cpp
 * @author Julian Oes <julian@oes.ch>
 * @license BSD 3-clause
 *
 * @brief ROS node to do offboard control of PX4 through MAVROS.
 *
 * Initial code taken from http://dev.px4.io/ros-mavros-offboard.html
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#define pi 3.1415926
double KPx = 0.6;
double KPy = 0.6;
double KPz = 0.7;
double KPyaw = 1;
double roll, pitch, yaw;
double roll2, pitch2, yaw2;
double yaw_desired;

using namespace std;
struct vir {
	double roll;
	double x;
	double y;
	double z;
};
struct displacement {
	double x;
	double y;
};


mavros_msgs::State current_state, current_state2;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr& msg)
{
	current_state2 = *msg;
}


geometry_msgs::PoseStamped host_mocap, host_mocap2;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap = *msg;
}

void host_pos2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	host_mocap2 = *msg;
}


// follow(vir1,uav0,&vs,dis1,uav1,dis2);
// vir        : position of virtual leader
// host_mocap : position of uav0
// vs         : velocity command of uav0
// dis_host   : desired displacement from uav0 to virtual leader
// nbr_mocap  : position of uav1
// dis_nbr    : desired displacement from uav1 to virtual leader

/*
 *
 *   o    o             o    o             o    o
 *    \  /               \  /               \  /
 *     \/                 \/                 \/
 *     /\                 /\                 /\
 *    /  \               /  \               /  \
 *   o    o             o    o             o    o
 *
 *   uav0(x1)     virtual leader(x0)      uav1(x1)
 *
 *   let the distance between virtual leader and uav0(uav1) be constant
 *   and give virtual leader the true desired trajectory,
 *   so the uavs will follow the virtual leader
 *
 * */
void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, displacement &dis_host, geometry_msgs::PoseStamped& nbr_mocap, displacement &dis_nbr, double yaw)
{
	double errx, erry, errz, err_yaw;
	double ux, uy, uz, u_yaw;
	double local_x, local_y;
	double local_x1, local_y1;
	double dis_x1, dis_y1;

	// convert dis1 from body frame to world frame
	local_x = cos(vir.roll)*dis_host.x+sin(vir.roll)*dis_host.y;
	local_y = -sin(vir.roll)*dis_host.x+cos(vir.roll)*dis_host.y;

	// calculate distance between uav0 and uav1
	dis_x1 = dis_host.x - dis_nbr.x;
	dis_y1 = dis_host.y - dis_nbr.y;

	// convert distance two uavs from body frame to world frame
	local_x1 = cos(vir.roll)*dis_x1+sin(vir.roll)*dis_y1;
	local_y1 = -sin(vir.roll)*dis_x1+cos(vir.roll)*dis_y1;

	// error = ( x0 - x1 + bias between one uav and virtual leader ) + ( x2 - x1 + bias between two uavs)
	errx = (vir.x - host_mocap.pose.position.x + local_x) + (nbr_mocap.pose.position.x - host_mocap.pose.position.x + local_x1);
	erry = (vir.y - host_mocap.pose.position.y + local_y) + (nbr_mocap.pose.position.y - host_mocap.pose.position.y + local_y1);
	errz = (vir.z - host_mocap.pose.position.z + 0) + (nbr_mocap.pose.position.z - host_mocap.pose.position.z + 0);
	err_yaw = yaw_desired - yaw;

	if(err_yaw > M_PI)
		err_yaw = err_yaw - 2*M_PI;
	else if(err_yaw < -M_PI)
		err_yaw = err_yaw + 2*M_PI;

	ux = KPx*errx;
	uy = KPy*erry;
	uz = KPz*errz;
	u_yaw = KPyaw*err_yaw;

	vs->twist.linear.x = ux;
	vs->twist.linear.y = uy;
	vs->twist.linear.z = uz;
	vs->twist.angular.z = u_yaw;
}

/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
	int flags = fcntl(0, F_GETFL, 0);
	fcntl(0, F_SETFL, flags | O_NONBLOCK);

	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0) {
		perror("tcsetattr()");
	}
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0) {
		perror("tcsetattr ICANON");
	}
	if (read(0, &buf, 1) < 0) {
		//perror ("read()");
	}
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0) {
		perror ("tcsetattr ~ICANON");
	}
	return (buf);
}
gazebo_msgs::LinkStates link_states;
geometry_msgs::PoseStamped uav0;
geometry_msgs::PoseStamped uav1;


void link_cb(const gazebo_msgs::LinkStates::ConstPtr& msg)
{

	link_states = *msg;
	if(link_states.name.size()>0) {
		for(unsigned int i=0; i<link_states.name.size(); i++) {
			if(link_states.name[i].compare("iris_0::base_link")==0) {
				uav0.pose.position.x = link_states.pose[i].position.x;
				uav0.pose.position.y = link_states.pose[i].position.y;
				uav0.pose.position.z = link_states.pose[i].position.z;
				roll = link_states.twist[i].angular.x;
				pitch = link_states.twist[i].angular.y;
				yaw = link_states.twist[i].angular.z;
				ROS_INFO("roll1 = %.2f, pitch1 = %.2f, yaw1 = %.2f", roll, pitch, yaw);
			}

			if(link_states.name[i].compare("iris_1::base_link")==0) {
				uav1.pose.position.x = link_states.pose[i].position.x;
				uav1.pose.position.y = link_states.pose[i].position.y;
				uav1.pose.position.z = link_states.pose[i].position.z;
				roll2 = link_states.twist[i].angular.x;
				pitch2 = link_states.twist[i].angular.y;
				yaw2 = link_states.twist[i].angular.z;
				ROS_INFO("roll1 = %.2f, pitch1 = %.2f, yaw1 = %.2f", roll, pitch, yaw);
			}
		}
	}
}
/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "formation_two_interaction");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
	                            ("uav0/mavros/state", 10, state_cb);
	ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>
	                             ("uav1/mavros/state", 10, state_cb2);
	//ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
	//                               ("drone2/mavros/mocap/pose", 1);
	//ros::Publisher mocap_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
	//                               ("drone3/mavros/mocap/pose", 1);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
	                                   ("uav0/mavros/cmd/arming");
	ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>
	                                    ("uav1/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
	                                     ("uav0/mavros/set_mode");
	ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>
	                                      ("uav1/mavros/set_mode");
	//ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody2/pose", 1, link_cb);

	//ros::Subscriber host_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody3/pose", 1, host_pos2);

	ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("uav0/mavros/setpoint_velocity/cmd_vel", 1);

	ros::Publisher local_vel_pub2 = nh.advertise<geometry_msgs::TwistStamped>("uav1/mavros/setpoint_velocity/cmd_vel", 1);
	ros::Subscriber link_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 3, link_cb);
	// The setpoint publishing rate MUST be faster than 2Hz.
	ros::Rate rate(50);

	// Wait for FCU connection.
	while (ros::ok() && current_state.connected && current_state2.connected) {
		//mocap_pos_pub.publish(host_mocap);
		//mocap_pos_pub2.publish(host_mocap2);
		ros::spinOnce();
		rate.sleep();
	}


	geometry_msgs::TwistStamped vs, vs2; // velocity command of uav0 and uav1
	vir vir1;
	displacement dis1,dis2; // displacement from uav0 to virtual leader , from uav1 to virtual leader

	dis1.x = -0.5;
	dis1.y = 0;
	dis2.x= 0.5;
	dis2.y = 0;

	vir1.x = 0.5;
	vir1.y = 0;
	vir1.z = 0.5;
	vir1.roll = 0;

	vs.twist.linear.x = 0;
	vs.twist.linear.y = 0;
	vs.twist.linear.z = 0;
	vs.twist.angular.x = 0;
	vs.twist.angular.y = 0;
	vs.twist.angular.z = 0;

	vs2.twist.linear.x = 0;
	vs2.twist.linear.y = 0;
	vs2.twist.linear.z = 0;
	vs2.twist.angular.x = 0;
	vs2.twist.angular.y = 0;
	vs2.twist.angular.z = 0;

	//send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i) {
		local_vel_pub.publish(vs);
		local_vel_pub2.publish(vs2);
		ros::spinOnce();
		rate.sleep();
	}

	mavros_msgs::SetMode offb_set_mode, offb_set_mode2;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	offb_set_mode2.request.custom_mode = "OFFBOARD";

	mavros_msgs::CommandBool arm_cmd, arm_cmd2;
	arm_cmd.request.value = true;
	arm_cmd2.request.value = true;

	ros::Time last_request = ros::Time::now();
	ros::Time last_request2 = ros::Time::now();
	//ros::Time last_request(0);

	while (ros::ok()) {
		if (current_state.mode != "OFFBOARD" &&
		    (ros::Time::now() - last_request > ros::Duration(5.0))) {
			if( set_mode_client.call(offb_set_mode) &&
			    offb_set_mode.response.mode_sent) {
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {

			if (!current_state.armed &&
			    (ros::Time::now() - last_request > ros::Duration(5.0))) {
				if( arming_client.call(arm_cmd) &&
				    arm_cmd.response.success) {
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		if (current_state2.mode != "OFFBOARD" &&
		    (ros::Time::now() - last_request2 > ros::Duration(5.0))) {
			if( set_mode_client2.call(offb_set_mode2) &&
			    offb_set_mode2.response.mode_sent) {
				ROS_INFO("Offboard2 enabled");
			}
			last_request2 = ros::Time::now();
		} else {

			if (!current_state2.armed &&
			    (ros::Time::now() - last_request2 > ros::Duration(5.0))) {
				if( arming_client2.call(arm_cmd2) &&
				    arm_cmd2.response.success) {
					ROS_INFO("Vehicle2 armed");
				}
				last_request2 = ros::Time::now();
			}
		}

		int c = getch();
		//ROS_INFO("C: %d",c);
		if (c != EOF) {
			switch (c) {
			case 65:    // key up (^)
				vir1.z += 0.1;
				break;
			case 66:    // key down (v)
				vir1.z += -0.1;
				break;
			case 67:    // key CW (>)
				vir1.roll -= 0.1;
				break;
			case 68:    // key CCW (<)
				vir1.roll += 0.1;
				break;
			case 119:    // key foward, move to x+ (w)
				vir1.x += 0.1;
				break;
			case 120:    // key back, move to x- (x)
				vir1.x += -0.1;
				break;
			case 97:    // key left, move to y+ (d)
				vir1.y += 0.1;
				break;
			case 100:    // key right, move to y- (a)
				vir1.y -= 0.1;
				break;
			case 115: {  // stop (s)
				vir1.x = 0;
				vir1.y = 0;
				vir1.z = 0;
				vir1.roll = 0;
				break;
			}
			case 108: {  // close arming (l)
				offb_set_mode.request.custom_mode = "MANUAL";
				offb_set_mode2.request.custom_mode = "MANUAL";
				set_mode_client.call(offb_set_mode);
				set_mode_client2.call(offb_set_mode2);
				arm_cmd.request.value = false;
				arm_cmd2.request.value = false;
				arming_client.call(arm_cmd);
				arming_client2.call(arm_cmd2);
				break;
			}
			case 63:
				return 0;
				break;
			}
		}

		//ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z,vir1.roll/pi*180);
		//ROS_INFO("roll2 = %.2f, pitch2 = %.2f, yaw2 = %.2f", roll2, pitch2, yaw2);
		follow(vir1,uav0,&vs,dis1,uav1,dis2,yaw);
		follow(vir1,uav1,&vs2,dis2,uav0,dis1,yaw2);
		local_vel_pub.publish(vs); // publish velocity command of uav0
		local_vel_pub2.publish(vs2); // publish velocity command of uav1

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
