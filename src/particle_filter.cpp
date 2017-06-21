#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include <nav_msgs/OccupancyGrid.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/Twist.h>

#define PI 3.1415926

bool PF_init, laser_is_set, map_is_set, laser_update;

sensor_msgs::PointCloud a;
sensor_msgs::LaserScan La, Lb;
nav_msgs::OccupancyGrid map_read;


class LaserModel {
	float range_min;
	float range_max;
	float angle_min;
	float angle_max;
	float angle_increment;
	int n;
	public:

	LaserModel (void) {}

	LaserModel( 
			float ang_min, float ang_max, float ang_inc, 
			float ran_min, float ran_max) 
	{
		range_min = ran_min;
		range_max = ran_max;
		angle_min = ang_min;
		angle_max = ang_max;
		angle_increment = ang_inc;
		float tmp = (angle_max-angle_min)/angle_increment;
		n = (unsigned) tmp;

		ROS_INFO("-------> %d", n);
		ROS_INFO("-------> %f %f %f", angle_min, angle_max, angle_increment);
		ROS_INFO("-------> %f ", tmp);
	}

	void setinfo ( sensor_msgs::LaserScan *L ) {
		L->range_min = range_min;
		L->range_max = range_max;
		L->angle_min =  angle_min;
		L->angle_max = angle_max;
		L->angle_increment = angle_increment;
	}

	void getinfo ( sensor_msgs::LaserScan L ) {
		range_min = L.range_min;
		range_max = L.range_max;
		angle_min = L.angle_min ;
		angle_max = L.angle_max ;
		angle_increment = L.angle_increment ;
	}

	unsigned getsize() {
		return n;
	}

	float getangle( unsigned i) {
		float tmp = angle_max-(i)*angle_increment;
		if ( tmp < angle_min )
			return angle_min;
		else if( tmp > angle_max )
			return angle_max;
		else
			return tmp;
	}

	float getlaser( unsigned i, float passo) {
		float tmp = range_min+i*passo;
		if ( tmp < range_max)
			return tmp;
		else
			return range_max;
	}

	unsigned get_count( float res) {
		float tmp = (range_max-range_min)/res;
		return tmp;
	}
};


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
	a.header = scan->header;
	Lb = *scan;
	ROS_INFO("Algo acontece aqui");
#if 0
	if ( laser_is_set ) {

		CompareLaser( La, *scan);
	}
	float angle;
	unsigned n = (scan->angle_max - scan->angle_min)/scan->angle_increment;

	for ( unsigned idx=0; idx<=n; idx++) {
		angle = scan->angle_max - idx*scan->angle_increment;

		ROS_INFO("-------> %f %f", (180*angle/PI), scan->ranges[idx]);
	}
#endif

}

void
GetFakeLaser ( 
		nav_msgs::OccupancyGrid map,
		geometry_msgs::Point32 p,
		LaserModel L, sensor_msgs::LaserScan *Lx)
{

	L.setinfo(Lx);

	unsigned map_size=map.info.width;
	// varios ranges

	float res = map.info.resolution;
	float comp=0;
	unsigned n = L.getsize();

//	ROS_INFO("Numero de feixes %d com %f de precisao", n, res);

	for ( unsigned idx=0; idx<n ; idx++) {

		float angle = L.getangle(idx);
		float laser = 0;
		unsigned m=L.get_count(res);
		float flag=0;

		for (  unsigned idx_laser=0;
				idx_laser<m;
				idx_laser++)
		{
			laser = L.getlaser(idx_laser, res);

			geometry_msgs::Point32 tmp;
			tmp.x=(p.x+cos(angle)*laser)/res;
			tmp.y=(p.y+sin(angle)*laser)/res;
			tmp.z=0;

			unsigned id_x = (unsigned) tmp.x;
			unsigned id_y = (unsigned) tmp.y;

			unsigned id = id_x+map_size*id_y;

//			ROS_INFO("M(%d,%d)=%d", id_x, id_y, id);

			int val= map.data[id];

//			ROS_INFO("M(%d,%d)=%d(%d)", id_x, id_y, id, val);
			if ( val > 0 ) {;
				flag=1;
				break;
			}
			if ( val < 0 ) {
				flag=-1;
				break;
			}
		}
//		ROS_INFO("%f, %f %d",
//			laser,
//			(180*angle/PI),
//			flag);
		if ( Lx->ranges.size() <= idx) {
			Lx->ranges.push_back(laser);
			Lx->intensities.push_back(flag);
		}
		else {
			Lx->ranges[idx]=laser;
//			ROS_INFO("----------- %f", flag);
			Lx->intensities[idx]=flag;
		}
	}

	laser_is_set=true;
}


float CompareLaser( sensor_msgs::LaserScan B, sensor_msgs::LaserScan A) {
	float angle;
	unsigned n = (A.angle_max - A.angle_min)/A.angle_increment;
	float diff, sum=0;

	for ( unsigned idx=0; idx<=n; idx++) {
		angle = A.angle_max - idx*A.angle_increment;

		if ( A.ranges[idx] > A.range_max )
			diff = (A.range_max - B.ranges[idx])/A.range_max;
		else 
			diff = (A.ranges[idx] - B.ranges[idx])/A.range_max;

		sum += fabs(diff);
	}
	sum /= n;
	return 1/sum;
}

void moveNuvem(sensor_msgs::PointCloud Xt, geometry_msgs::Twist ut,
		ros::Duration t) {
	int n=Xt.points.size();
	for ( int i=0; i<n; i++) {
		Xt.points[i].x += ut.linear.x*t.toSec();
		Xt.points[i].y += ut.linear.y*t.toSec();
		Xt.points[i].z += ut.angular.z*t.toSec();
	}
}

void Resample(sensor_msgs::PointCloud Xt);


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "my_scan_to_cloud");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	if ( !PF_init ) {
		a.header.seq = 1;
		a.header.stamp =  ros::Time::now();
		a.header.frame_id = "base_link";
		sensor_msgs::ChannelFloat32 cha;
		geometry_msgs::Point32 p;
		float values;

		cha.name="peso";
		cha.values.push_back(1);
		float res_x=0.1;
		float res_y=0.1;

		for ( int i=0; i<20; i++) {
			p.x=i*res_x-9.5*res_x;
			for ( int j=0; j<20; j++) {
				p.y=j*res_y-9.5*res_y;
				p.z=0;
				a.points.push_back(p);
				a.channels.push_back(cha);
			}
		}
		PF_init=true;
	}

	// comparar a leitura do sensor com a 
	// Pegar um mapa
	std::string map_top="/map";
	map_read=*(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_top, ros::Duration(10)));

	ros::Publisher chatter_pub = 
		n.advertise<sensor_msgs::PointCloud>("/my_acml/pointcloud", 1000);
	
	std::string scan_top="/scan";
	Lb=*(ros::topic::waitForMessage<sensor_msgs::LaserScan>(scan_top, ros::Duration(10)));

	LaserModel L(-3*PI/4, 3*PI/4, 3*PI/(2*720), 0.01, 25.0);

	std::string flag;
	while(ros::ok() ) {
	
		// Simular a leitura do laser
		geometry_msgs::Point32 p;
		p.x = -map_read.info.origin.position.x;
		p.y = -map_read.info.origin.position.y;
		p.z = 0;
		p.x += 0.337;

		int n=a.points.size();
		for ( unsigned i=0; i<n; i++) {

			ROS_INFO("Pointo[%d]=(%f, %f, %f)", i, a.points[i].x, a.points[i].y,
					a.points[i].z);
			geometry_msgs::Point32 sample=p;
			sample.x +=a.points[i].x;
			sample.y +=a.points[i].y;
			sample.z += 0;

			GetFakeLaser(map_read, sample, L, &La);
			float tmp = CompareLaser(La, Lb);
			ROS_INFO("Likelihood(%d)=%f", i, tmp);
		}

		chatter_pub.publish(a);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
