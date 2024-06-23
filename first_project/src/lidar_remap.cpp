#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>
#include <tf/transform_listener.h>

class LidarRemapNode{
public:
	LidarRemapNode(){
		// set dynamic reconfigure callback
		f = boost::bind(&LidarRemapNode::reconfigurecallback, this, _1, _2);
		server.setCallback(f);
		
		// set subscriber and publisher
		lidar_sub = nh.subscribe("os_cloud_node/points", 1000, &LidarRemapNode::lidarcallback, this);
		lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1000);
		
	}
	
	void lidarcallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
		sensor_msgs::PointCloud2 transformed_msg;
		
		// modify frame in the header
		transformed_msg = *msg;
		transformed_msg.header.frame_id = frame_id;
		
		// publish the remapped point cloud on a new topic
		lidar_pub.publish(transformed_msg);
		
	}
	
	void reconfigurecallback(first_project::parametersConfig& config, uint32_t level){
		// change frame_id based on dynamic parameter
		
		switch(config.target_frame_id){
			case 0: frame_id = "wheel_odom"; break;
			case 1: frame_id = "gps_odom"; break;
		}
	}
	
private:
	ros::NodeHandle nh;
	tf::TransformListener tf_listener;
	dynamic_reconfigure::Server<first_project::parametersConfig> server;
	dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
	std::string frame_id;
	ros::Subscriber lidar_sub;
	ros::Publisher lidar_pub;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "lidar_remap");
	LidarRemapNode node;
	ros::spin();
	return 0;
}
