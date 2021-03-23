/**
 * @file pos_ctrl_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


struct Quaternion
{
    double w, x, y, z;
};

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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_ctrl_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();

    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Quaternion Q_desired;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    Q_desired.x = 0;
    Q_desired.y = 0;
    Q_desired.z = 0;
    Q_desired.w = 1;
    Quaternion q;

    float yaw_desired;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        int c = getch();
        if (c != EOF)
        {
            switch (c)
            {
                case 65:    // key up 65 (v)
                    //vir1.z += 0.05;
                    pose.pose.position.z +=0.05;
                    break;
                case 66:    // key down (^)
                    //vir1.z += -0.05;
                    pose.pose.position.z -=0.05;
                    break;

                case 67:
                    yaw_desired -= 0.03;
                    q = ToQuaternion(yaw_desired, 0, 0);
                    Q_desired.x = q.x;
                    Q_desired.y = q.y;
                    Q_desired.z = q.z;
                    Q_desired.w = q.w;
                    break;

                case 68:
                    yaw_desired += 0.03;
                    q = ToQuaternion(yaw_desired, 0, 0);
                    Q_desired.x = q.x;
                    Q_desired.y = q.y;
                    Q_desired.z = q.z;
                    Q_desired.w = q.w;
                    break;

                case 100: // move to y- (a)
                	pose.pose.position.y -= 0.05;
                	break;

                case 97: // move to y+ (d)
                	pose.pose.position.y += 0.05;
                	break;

                case 119: //move to x+ (w)
                	pose.pose.position.x += 0.05;
                	break;

                case 120: // move to x- (x)
                	pose.pose.position.x -= 0.05;
                	break;

                case 115:    // stop 115 (s)
                    {
                        //vir1.x = 0.6;
                        //vir1.y = -0.5;
                        //vir1.z = 0;
                        pose.pose.position.z = 0;
                        //vir1.roll = 0;
                        break;
                    }
                case 108:    // close arming (l)
                    {
                        offb_set_mode.request.custom_mode = "MANUAL";
                        set_mode_client.call(offb_set_mode);
                        arm_cmd.request.value = false;
                        arming_client.call(arm_cmd);
                        break;
                     }
                case 63:
                       return 0;
                       break;
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
