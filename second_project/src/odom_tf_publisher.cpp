#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	
	// set the transform based on the odometry message
	transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	transform.setRotation(q);
	
	// broadcast the tf
	br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "odom_tf_publisher");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/ugv/odom", 100, odomCallback);
	
	ros::spin();
	return 0;
}
