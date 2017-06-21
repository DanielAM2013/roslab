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
	return -1;
}


float Theta_d( float x, float y) {
	if ( x==0 )
		x=0.01;

	return 2*atan(y/x);
}

float NormRad( float ang) {
	while(ang<-pi) {
		ang+=2*pi;
	}
	while(ang>pi) {
		ang-=2*pi;
	}
	return ang;
}

int main ( int argc, char **argv) {

	ros::init(argc, argv, "atratores");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	ros::Subscriber sub = n.subscribe("/turtle1/pose", 1000, chatterCallback);
	ros::Rate loop_rate(10);

	bool state;
	float p_x, p_y, p_theta;
	ROS_INFO("Coordenada x: ");
	std::cin >> p_x;
	ROS_INFO("Coordenada y: ");
	std::cin >> p_y;
	ROS_INFO("Coordenada theta: ");
	std::cin >> p_theta;

	ROS_INFO("P(%f,%f,%f)",dx- p_x, dy-p_y, da-p_theta);

	float k=1, gamma=1;
#if 0
	ROS_INFO("Parametro k: ");
	std::cin >> k;
	ROS_INFO("Parametro b: ");
	std::cin >> gamma;
#endif
	float erro_x, erro_y, erro_a;
	float y_m, x_m;
	float theta_d, alpha, beta, a;
	float b_1, b_2;
	while (ros::ok()) {

		if (erro_x*erro_x+erro_y*erro_y<1e-4) {
			ROS_INFO("Coordenada x: ");
			std::cin >> p_x;
			ROS_INFO("Coordenada y: ");
			std::cin >> p_y;
			ROS_INFO("Coordenada theta: ");
			std::cin >> p_theta;
		}

		erro_x=dx - p_x;
		erro_y=dy - p_y;
		erro_a=da - p_theta; 

		ROS_INFO("P[%f][%f][%f]",erro_x, erro_y, erro_a);

		x_m = cos(p_theta)*erro_x+sin(p_theta)*erro_y;
		y_m =-sin(p_theta)*erro_x+cos(p_theta)*erro_y;

		beta=y_m/x_m;

		theta_d = Theta_d(x_m, y_m);

		a=Sign(x_m,y_m)*sqrt(x_m*x_m+y_m*y_m)/Sinc(theta_d/2.0);

		alpha=NormRad(erro_a - theta_d);

		b_1 = cos(erro_a)*(theta_d/beta - 1)+
			sin(erro_a)*((theta_d/2)*(1-1/(beta*beta))+1/beta);

		b_2 =-sin(erro_a)*(2/((1+beta*beta)*erro_x))
			+ cos(erro_a)*(2*beta/((1+beta*beta)*erro_x));

		ROS_INFO("(b_1,b_2): [%f][%f]", b_1, b_2);
		ROS_INFO("(alpha,beta): [%f][%f]", alpha, beta);

		geometry_msgs::Twist msg;

		msg.linear.x=-gamma*b_1*a/(1+abs(a));
		msg.angular.z=-b_2*msg.linear.x-(k*alpha);

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;

}
