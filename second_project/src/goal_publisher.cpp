#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <fstream>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal{
	double x, y, theta;
};

std::vector<Goal> readGoals(const std::string& filename){
	std::vector<Goal> goals;
	std::ifstream file(filename);
	std::string line;
	while (std::getline(file, line)){
		std::istringstream iss(line);
		Goal goal;
		char comma;
		iss >> goal.x >> comma >> goal.y >> comma >> goal.theta;
		goals.push_back(goal);
	}
	return goals;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "goal_publisher");
	ros::NodeHandle nh;
	
	std::string file_name;
	nh.getParam("goal_publisher/waypoints", file_name);
	ROS_INFO("Opening waypoints file...");
	
	MoveBaseClient ac("move_base", true);
	ROS_INFO("Waiting for server...");
	ac.waitForServer();
	
	std::vector<Goal> goals = readGoals(file_name);
	
	for (const auto& goal : goals){
		move_base_msgs::MoveBaseGoal move_base_goal;
		move_base_goal.target_pose.header.frame_id = "map";
		move_base_goal.target_pose.header.stamp = ros::Time::now();
		move_base_goal.target_pose.pose.position.x = goal.x;
		move_base_goal.target_pose.pose.position.y = goal.y;
		move_base_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal.theta);
		
		ROS_INFO("Sending goal: x=%f, y=%f, theta=%f", goal.x, goal.y, goal.theta);
		ac.sendGoal(move_base_goal);
		
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Goal reached.");
			} else {
				ROS_INFO("Goal Failed");
			}
	}
	return 0;
}
