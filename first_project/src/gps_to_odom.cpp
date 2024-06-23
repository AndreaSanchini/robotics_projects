#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>



class GPSToOdom {
public:
	GPSToOdom(){
	
	// params
	nh.getParam("lat_r", lat_r);
	nh.getParam("lon_r", lon_r);
	nh.getParam("alt_r", alt_r);
	
	// initialize sub and pub
	gps_sub = nh.subscribe("fix", 1000, &GPSToOdom::gpscallback, this);
	odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1000);
	}
	
	void gpscallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
		// get timestamp 
		ros::Time stamp = msg->header.stamp;
		
		// perform gps->ecef conversion
		double x, y, z;
		gpstoecef(msg->latitude, msg->longitude, msg->altitude, x, y, z);
		
		// perform ecef->enu conversion
		eceftoenu(x, y, z, x, y, z);
		
		// pub odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = stamp;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = z;
		
		// estimate heading using consecutive poses
		tf::Quaternion q;
		if(first_pose_received){
			double dx = x - last_x;
			double dy = y - last_y;
			double yaw = atan2(dy, dx);
			//ROS_INFO("heading diretions is %f", yaw * 180.0 / M_PI);
			q.setRPY(0, 0, yaw);
			tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
		}
		else{
			q.setRPY(0, 0, 0);
			tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
			first_pose_received = true;
		}
		last_x = x;
		last_y = y;
		odom_pub.publish(odom);
	}
	
	void gpstoecef(double lat, double lon, double alt, double& x, double& y, double& z){
		// convert gps to ecef
		const double a = 6378137.0;
		const double b = 6356752.0;
		double e_2 = 1 - pow(b, 2) / pow(a, 2);
		
		// convert lat and long from deg to rad
		double lat_rad = lat * M_PI / 180.0;
		double lon_rad = lon * M_PI / 180.0;
		
		// compute N 
		double N = a / sqrt(1.0 - e_2 * pow(sin(lat_rad), 2));
		
		// get x, y, z in ecef
		x = (N + alt) * cos(lat_rad) * cos(lon_rad);
		y = (N + alt) * cos(lat_rad) * sin(lon_rad);
		z = (N * (1 - e_2) + alt) * sin(lat_rad);
	}
	
	void eceftoenu(double x_ecef, double y_ecef, double z_ecef, double& x_enu, double& y_enu, double& z_enu){
		double x0, y0, z0;
		gpstoecef(lat_r, lon_r, alt_r, x0, y0, z0);
		double dx = x_ecef - x0;
		double dy = y_ecef - y0;
		double dz = z_ecef - z0;
		
		
		// rotation matrix
		double slat = sin(lat_r * M_PI / 180.0);
		double clat = cos(lat_r * M_PI / 180.0);
		double slon = sin(lon_r * M_PI / 180.0);
		double clon = cos(lon_r * M_PI / 180.0);
		double R[3][3] = {
			{-slon, clon, 0},
			{-slat * clon, -slat * slon, clat},
			{clat * clon, clat * slon, slat}
		};
		
		x_enu = R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
		y_enu = R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
		z_enu = R[2][0] * dx + R[2][1] * dy + R[2][2] * dz;
		
		// rotate the enu values (because there'an offset between gps and wheel odom)
		double x_rot, y_rot;
		x_rot = x_enu * cos(130.0 * M_PI / 180.0) - y_enu * sin(130.0 * M_PI / 180.0);
		y_rot = x_enu * sin(130.0 * M_PI / 180.0) + y_enu * cos(130.0 * M_PI / 180.0);
		x_enu = x_rot;
		y_enu = y_rot;
		
	}
private:
	ros::NodeHandle nh;
	ros::Subscriber gps_sub;
	ros::Publisher odom_pub;
	double lat_r, lon_r, alt_r;
	double last_x, last_y;
	bool first_pose_received = false;

};

int main(int argc, char** argv){
	ros::init(argc, argv, "gps_to_odom");
	GPSToOdom gps_to_odom;
	ros::spin();
	return 0;
}	
