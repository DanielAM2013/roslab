#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <math.h>
#include <iostream>
using namespace std;

double coord_x=0.0, coord_y=0.0, angulo = 0.0, gama = 0.5;
double k = 1;

double Teta_d(double x, double y)
{
    if(x==0.0)
        x = 0.1;
    return (2.0*atan(y/x));
}

double NormRad(double ang){
	while(ang>M_PI)
		ang = ang-2*M_PI;
	while(ang<-M_PI)
		ang = ang+2*M_PI;

return(ang);
}

double SinC(double theta){
    double sinc = 1.0;

    if (abs(theta) > 0.001)
        return(sin(theta)/theta);

    return sinc;
}

double Sign(double x, double y){
   if((x==0.0 && y<0.0) || x>0.0)
        return 1.0;
    
    return -1.0;
}

double funcB1(double x, double y, double theta, double alpha){
    double mybeta = y/x;
    double theta_d = Teta_d(x,y);

    if (theta_d != 0)	
        return cos(theta)*((theta_d/mybeta) - 1.0) + sin(theta)*(theta_d/2.0*(1.0-1.0/pow(mybeta,2.0)) + 1.0/mybeta);
    return cos(alpha);    
}

double funcB2(double x, double y, double theta){
    double mybeta = y/x;
    double theta_d = Teta_d(x,y);

    if (theta_d != 0)
        return cos(theta)*(2.0*mybeta/((1.0+pow(mybeta,2.0))*x)) - sin(theta)*(2.0/((1.0+pow(mybeta,2.0))*x));
    return sin(theta_d/2.0 - theta) * (2.0 / SinC(theta_d/2.0));
}


void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

	coord_x = msg->pose.pose.position.x;
	coord_y = msg->pose.pose.position.y;	
	angulo = tf::getYaw(msg->pose.pose.orientation);
}

double graus(double grau){
	return(grau*M_PI/180);
}


int main(int argc, char **argv){

	double Ex=0, Ey=0, Eteta=0, xM=0, yM=0, a=0, alpha=0, beta=0, fb1=0, fb2=0;
	double u=0, w=0, Erro_x=0, Erro_y=0, Erro_teta=0, teta_d=0;	
	
	ros::init(argc, argv, "Controle_P3AT");
	ros::NodeHandle n;
		
	//recupera os dados da odometria pela função
	ros::Subscriber sub = n.subscribe("/RosAria/pose",1000, chatterCallback);
	
	geometry_msgs::Twist vel_msg;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",100);

	ROS_INFO("Coordenada x:");
	cin >> Erro_x;
	
	ROS_INFO("Coordenada y:");
	cin >> Erro_y;

	do{
		ROS_INFO("Theta em Graus (-180 <> 180):");
		cin >> Erro_teta;

		if(Erro_teta<-180 || Erro_teta>180)
			ROS_INFO("Valor invalido!");
	
	}while((Erro_teta<-180) || (Erro_teta>180));
	
	Erro_teta=graus(Erro_teta);


	while(ros::ok()){
		
		Ex = coord_x - Erro_x;
		Ey = coord_y - Erro_y;
		Eteta = angulo - Erro_teta;

		ROS_INFO("Local -> Eixo_X: [%f],Eixo_Y: [%f], Angulo: [%f]", coord_x,coord_y,angulo);

		xM = (cos(Erro_teta)*Ex + sin(Erro_teta)*Ey);
		yM = (-sin(Erro_teta)*Ex + cos(Erro_teta)*Ey);

		a = Sign(xM,yM)*sqrt(xM*xM + yM*yM)/(SinC(Teta_d(xM,yM)/2.0));
		alpha = NormRad(Eteta-Teta_d(xM,yM));

		fb1 = funcB1(xM,yM,Eteta, alpha);
		u = -gama/(1.0 + abs(a))*fb1*a;

        	fb2 = funcB2(xM,yM,Eteta);
		w = (-fb2*u) - (k*alpha);
		

		ROS_INFO("u, w: [%f, %f]",u,w);

		vel_msg.linear.x = u; //seta a velocidade linear no eixo x
		
		vel_msg.angular.z = w; //seta o valor de rotação
	
		velocity_publisher.publish(vel_msg);
				
		ros::spinOnce();
	}

	ros::spin();
	return 0;
}
