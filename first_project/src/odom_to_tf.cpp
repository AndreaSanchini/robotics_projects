#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomToTfNode {
public: 
	OdomToTfNode() : nh("~") {
		//parameters
		nh.getParam("root_frame", root_frame);
		nh.getParam("child_frame", child_frame);
		
		// subscriber
		odom_sub = nh.subscribe("input_odom", 1000, &OdomToTfNode::odomcallback, this);
	}
	
	void odomcallback(const nav_msgs::Odometry::ConstPtr& msg){
		// get timestamp 
		ros::Time stamp = msg->header.stamp;
		
		// publish the tf transform
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
		tf::Quaternion q;
		tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
		transform.setRotation(q);
		
		tf_broadcaster.sendTransform(tf::StampedTransform(transform, stamp, root_frame, child_frame));
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	tf::TransformBroadcaster tf_broadcaster;
	std::string root_frame, child_frame;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "odom_to_tf");
	OdomToTfNode node;
	ros::spin();
	return 0;
}
