#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/LinearMath/Quaternion.h>

#include <vector>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using move_base_msgs::MoveBaseGoal;
using std::vector;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_point");
  ros::NodeHandle nh("~");

  int num_points;
  vector<double> x_coords, y_coords, theta_coords;
  nh.getParam("num_points", num_points);
  nh.getParam("points_x", x_coords);
  nh.getParam("points_y", y_coords);
  nh.getParam("points_theta", theta_coords);
  ROS_INFO("get param %f", x_coords[0]);
  if(x_coords.size() != num_points || y_coords.size() != num_points || theta_coords.size() != num_points){
    ROS_ERROR("Wrong Point Number!");
    return 0;
  }
  
  vector<geometry_msgs::Pose2D> goal_poses;
  geometry_msgs::Pose2D tmp_pose;
  for(auto i = 0; i < num_points; ++i){
    tmp_pose.x = x_coords[i];
    tmp_pose.y = y_coords[i];
    tmp_pose.theta = theta_coords[i];
    goal_poses.push_back(tmp_pose);
  }

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  MoveBaseGoal curr_goal;
  curr_goal.target_pose.header.frame_id = "base_link";

  geometry_msgs::Pose2D curr_pose;
  size_t point_states = 0;

  tf::Quaternion q;
  while(point_states < num_points){
    curr_pose = goal_poses[point_states];
    curr_goal.target_pose.header.stamp = ros::Time::now();
    curr_goal.target_pose.pose.position.x = curr_pose.x;
    curr_goal.target_pose.pose.position.y = curr_pose.y;
    q.setRPY(0, 0, curr_pose.theta);
    curr_goal.target_pose.pose.orientation.w = q.w();
    curr_goal.target_pose.pose.orientation.x = q.x();
    curr_goal.target_pose.pose.orientation.y = q.y();
    curr_goal.target_pose.pose.orientation.z = q.z();

    ac.sendGoal(curr_goal);
    ac.waitForServer();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ++point_states;
    } else {
      ROS_WARN("Fail to go to target point");
    }
    if(!ros::ok()) break;
  }
  return 0;
  
}