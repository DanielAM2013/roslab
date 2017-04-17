#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iostream>
#include <cmath>
#include <tf/tf.h>

#define pi 3.1415
float dx, dy, da;

void chatterCallback(const turtlesim::Pose::ConstPtr& msg) {

	dx=msg->x;
	dy=msg->y;
	da=msg->theta;
	ROS_INFO("I heard: [%f][%f][%f]", dx, dy, da);
}

float Sinc( float ang) {
	if ( abs(ang) > 0.001 )
		return sin(ang)/ang;
	else
		return 1;
}

int Sign(float x, float y) {
	if ( x > 0 || (x==0 && y<0) )
		return 1;
	else
		return -1;
}


float NormRad( float ang) {
	while(ang<pi) {
		ang+=2*pi;
	}
	while(ang>pi) {
		ang-=2*pi;
	}
	return ang;
}


float Theta_d( float x, float y) {
	float theta_d;
	if (!x && !y) {
		theta_d=0;
	}
	else
		theta_d = 2*atan2(y,x);
	return NormRad(theta_d);
}


int main ( int argc, char **argv) {

	ros::init(argc, argv, "atratores");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	ros::Subscriber sub = n.subscribe("/turtle1/pose", 1000, chatterCallback);
	ros::Rate loop_rate(10);

	float p_x, p_y;
	ROS_INFO("Coordenada x: ");
	std::cin >> p_x;
	ROS_INFO("Coordenada y: ");
	std::cin >> p_y;

	float k, b;
	ROS_INFO("Parametro k_1: ");
	std::cin >> k;
	ROS_INFO("Parametro k_2: ");
	std::cin >> b;

	float theta_d, alpha, beta, a;
	float b_1, b_2;
	while (ros::ok()) {

		beta=dy/dx;
		theta_d = Theta_d(dx, dy);
		a=sqrt(dx*dx+dy*dy)/Sinc(theta_d/2)*Sign(dx,dy);
		alpha=da - theta_d;
		alpha=NormRad(alpha);
		b_1 = cos(da)*(theta_d/beta - 1)+
			sin(da)*((theta_d/2)*(1-1/(beta*beta))+1/beta);

		b_2 = cos(da)*((2*beta)/(1+beta*beta)*dx)-sin(da)*
			((2*beta)/(1+beta*beta)*dx);

		geometry_msgs::Twist msg;

		msg.linear.x=-b*b_1*a;
		msg.angular.z=(-b_2*msg.linear.x)-(k*alpha);

		ROS_INFO("(b_1,b_2): [%f][%f]", b_1, b_2);
		ROS_INFO("(alpha,beta): [%f][%f]", alpha, beta);

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;

}
