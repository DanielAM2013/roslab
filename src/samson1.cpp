#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iostream>
#include <cmath>
#include <tf/tf.h>

float d=0.25;
float x, y, theta;
float x_0, y_0;
bool is_origin_set;

void chatterCallback(const turtlesim::Pose::ConstPtr& msg) {

	if (!is_origin_set) {
		x_0=msg->x;
		y_0=msg->y;
		is_origin_set=true;
	}

	x=msg->x-x_0;
	y=msg->y-y_0;
	theta=msg->theta;
	ROS_INFO("I heard: [%f][%f][%f]", x+d, y, theta);
}

int main ( int argc, char **argv) {

	ros::init(argc, argv, "samson1");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	ros::Subscriber sub = n.subscribe("/turtle1/pose", 1000, chatterCallback);
	ros::Rate loop_rate(10);


	int count = 0;

	ROS_INFO("I heard: [%f][%f][%f]", x+d, y, theta);
	float p_x, p_y;
	ROS_INFO("Coordenada x: ");
	std::cin >> p_x;
	ROS_INFO("Coordenada y: ");
	std::cin >> p_y;

	float k_1, k_2, a;
	ROS_INFO("Parametro k_1: ");
	std::cin >> k_1;
	ROS_INFO("Parametro k_2: ");
	std::cin >> k_2;
	ROS_INFO("Parametro a: ");
	std::cin >> a;

	float e_x, e_y;
	float x_m, y_m;

	while (ros::ok()) {
//		std_msgs::String msg;

		e_x = p_x - x;
		e_y = p_y - y;

		x_m = cos(theta)*e_x+sin(theta)*e_y;
		y_m = -sin(theta)*e_x+cos(theta)*e_y;
		x_m-= d;
		geometry_msgs::Twist msg;
//		std::stringstream ss;

		msg.linear.x=k_1/(1/a+abs(x_m))*x_m;
		msg.angular.z=k_2/(1/a+abs(y_m))*y_m;

		ROS_INFO("(x_m,y_m,e_x,e_y): [%f][%f][%f][%f]", x_m, y_m, e_x, e_y);

//		ss << "hello world" << count;
//		msg.data = ss.str();

		

//		ROS_INFO("%f", msg.linear.x);

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}
