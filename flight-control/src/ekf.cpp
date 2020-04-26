#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <flight_control/qptrajectory.h>
#define gravity 9.806
#define pi 3.1415926
bool init = false;
bool initial = false;
bool start = false;
int flag=0;
float KPx=1, KPy=1, KPz=1;
//float KPx=5, KPy=5, KPz=1;
float KPyaw = 1;
double roll, pitch, yaw;
float r = 0.5;
float T = 2*pi;
using namespace std;
typedef struct
{
    float roll;
    float x;
    float y;
    float z;
}vir;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
nav_msgs::Odometry cam_imu;
void extrinsic(const nav_msgs::Odometry::ConstPtr& msg)
{
    cam_imu = *msg;
}
geometry_msgs::Twist host_vel;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;
void host_pos(const nav_msgs::Odometry::ConstPtr& msg)
{
        host_mocap.header = msg->header;
        host_mocap.pose.position = msg->pose.pose.position;
        host_mocap.pose.orientation = msg->pose.pose.orientation;
//        host_vel = msg->twist.twist;
        if(init==false)
        {
        initial_pose = host_mocap;
        init = true;
        }
        tf::Quaternion Q(
            host_mocap.pose.orientation.x,
            host_mocap.pose.orientation.y,
            host_mocap.pose.orientation.z,
            host_mocap.pose.orientation.w);

        tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}
void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs,float vx,float vy,float ax,float ay)
{
float errx, erry, errz, err_roll;
float ux, uy, uz, uroll;

errx = vir.x - host_mocap.pose.position.x ;
erry = vir.y - host_mocap.pose.position.y ;
errz = vir.z - host_mocap.pose.position.z ;
err_roll = vir.roll - yaw;
if(err_roll>pi)
err_roll = err_roll - 2*pi;
else if(err_roll<-pi)
err_roll = err_roll + 2*pi;

ROS_INFO("err: %.3f,%.3f,%.3f,%.3f",errx,erry,errz,err_roll);
if(start == false){
    ux = KPx*errx;
    uy = KPy*erry;
    uz = KPz*errz;
    uroll = KPyaw*err_roll;
}
else{
    ux = KPx*errx + vx;
    uy = KPy*erry + vy;
    uz = KPz*errz;
    uroll = KPyaw*err_roll;
}

if(ux<=-1.5 ||ux>=1.5)
{
ux = 1.5*ux/abs(ux);
}
if(uy<=-1.5 ||uy>=1.5)
{
uy = 1.5*uy/abs(uy);
}
if(uz<=-0.4 ||uz>=0.4)
{
uz = 0.4*uz/abs(uz);
}

vs->twist.linear.x = ux;
vs->twist.linear.y = uy;
vs->twist.linear.z = uz;
vs->twist.angular.z = uroll;

}

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

/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "track");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/mavros/setpoint_position/local", 10);
    //ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   //("/mavros/mocap/pose", 2);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");
    //ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose", 10, host_pos);
    ros::Subscriber host_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_estimator/imu_propagate",2, host_pos);
    ros::Subscriber ex_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_estimator/extrinsic",2, extrinsic);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 2);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("/desired_position", 2);
//    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("/desired_velocity", 2);
//    ros::Publisher mv_pub = nh.advertise<geometry_msgs::Point>("/motive_vel",2);
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(50);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
	//mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::Point desired_pos;
    geometry_msgs::Point desired_vel;

    vir vir1;
    float r = 0.5;
    float T = 2*pi;
    float dt = 0.02;
    float t = 0;
    vir1.x = 0;
    vir1.y = 0;
    vir1.z = 0.5;
    vir1.roll = 0;

    geometry_msgs::TwistStamped vs;
    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;

    geometry_msgs::Point pos;
    geometry_msgs::Point vel;
    geometry_msgs::Point acc;
    double max=0;
    double count=0;
    qptrajectory plan;
    path_def path;
    trajectory_profile p1,p2,p3,p4,p5;
    std::vector<trajectory_profile> data;
    p1.pos << 0,0,0;
    p1.vel << 0.0,0.0,0;
    p1.acc << 0.00,-0.0,0;
    p1.yaw = 0;

    p2.pos<< 0.6,-0.6,0;
    p2.vel<< 0,0,0;
    p2.acc<< 0,0,0;
    p2.yaw = 0;

    p3.pos<< 0,0.6,0;
    p3.vel<< 0,0,0;
    p3.acc<< 0,0,0;
    p3.yaw = 0;

    p4.pos << -0.6,-0.6,0;
    p4.vel << 0,0,0;
    p4.acc << 0,0,0;
    p4.yaw = 0;

    p5.pos << 0,0,0;
    p5.vel << 0.0,0.0,0;
    p5.acc << 0.00,-0.0,0;
    p5.yaw = 0;

   path.push_back(segments(p1,p2,3));
   path.push_back(segments(p2,p3,3));
   path.push_back(segments(p3,p4,3));
   path.push_back(segments(p4,p5,3));

    data = plan.get_profile(path ,path.size(),0.02);
    max = data.size();
    //send a few setpoints before starting
   for(int i = 300; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vs);
        //mocap_pos_pub.publish(host_mocap);
        vir1.x = initial_pose.pose.position.x;
        vir1.y = initial_pose.pose.position.y;
        vir1.z = initial_pose.pose.position.z+0.5;
        vir1.roll = yaw;
        ROS_WARN("extrinsic:%.2f, %.2f, %.2f",cam_imu.pose.pose.position.x,cam_imu.pose.pose.position.y,cam_imu.pose.pose.position.z);

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
	//ros::Time last_request(0);

    while (ros::ok())
    {
	//mocap_pos_pub.publish(host_mocap);
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        int c = getch();
		//ROS_INFO("C: %d",c);
        if (c != EOF)
        {
            switch (c) {
            case 65:    // key up
                vir1.z += 0.05;
                break;
            case 66:    // key down
                vir1.z += -0.05;
                break;
            case 67:    // key CW(->)
                vir1.roll -= 0.03;
                break;
            case 68:    // key CCW(<-)
                vir1.roll += 0.03;
                break;
			case 119:    // key foward
                pos.x += 0.05;
                break;
            case 120:    // key back
                pos.x += -0.05;
                break;
            case 97:    // key left
                pos.y += 0.05;
                break;
            case 100:    // key right
                pos.y -= 0.05;
                break;
	    	case 115:    // key right
		{
                vir1.x = initial_pose.pose.position.x;
                vir1.y = initial_pose.pose.position.y;
                vir1.z = initial_pose.pose.position.z+0.2;
                break;
		}
		case 108:    // close arming
			{
			offb_set_mode.request.custom_mode = "MANUAL";
			set_mode_client.call(offb_set_mode);
			arm_cmd.request.value = false;
			arming_client.call(arm_cmd);
            break;
			}
            case 116:    // t
            {
                if(start == true){
                start = false;
                }
                else if(start == false){
                start = true;
                }
                count = 0;
            break;
            }
            case 105:    // i
            {
                if(initial == true){
                initial = false;
                }
                else if(initial == false){
                initial = true;
                }

            break;
            }
            case 63:
                return 0;
                break;
            }
        }
		if(vir1.roll>pi)
		vir1.roll = vir1.roll - 2*pi;
		else if(vir1.roll<-pi)
		vir1.roll = vir1.roll + 2*pi;

        if(initial == true)
        {
            vir1.y = initial_pose.pose.position.y + r*sin(2*pi*t/T);
            t += dt;
        }
        ROS_INFO("%d,%2f,%2f",start,count,max);
        if(start == true)
        {
        //circle
//        vir1.x = initial_pose.pose.position.x + r*cos(2*pi*t/T);
//        vir1.y = initial_pose.pose.position.y + r*sin(2*pi*t/T);
//            t += dt;
            pos.x = data[count].pos[0];
            pos.y = data[count].pos[1];
            pos.z = data[count].pos[2];

            vel.x = data[count].vel[0];
            vel.y = data[count].vel[1];
            vel.z = data[count].vel[2];

            acc.x = data[count].acc[0];
            acc.y = data[count].acc[1];
            acc.z = data[count].acc[2];
            vir1.x = initial_pose.pose.position.x + pos.x;
            vir1.y = initial_pose.pose.position.y + pos.y;

            count++;
            if(count >=max){
                start = false;
                pos.x = 0;
                pos.y = 0;
                pos.z = 0;
            }
        }
        else
        {
            vel.x = 0;
            vel.y = 0;
            vel.z = 0;

            acc.x = 0;
            acc.y = 0;
            acc.z = 0;
            vir1.x = initial_pose.pose.position.x + pos.x;
            vir1.y = initial_pose.pose.position.y + pos.y;
        }
        //ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.roll/pi*180);
        follow(vir1,host_mocap,&vs,vel.x,vel.y,0,0);
        desired_pos.x = vir1.x;
        desired_pos.y = vir1.y;
        desired_pos.z = vir1.z;
        //mocap_pos_pub.publish(host_mocap);
        local_vel_pub.publish(vs);
        pos_pub.publish(desired_pos);
//        vel_pub.publish(vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



