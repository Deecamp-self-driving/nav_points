#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <vector>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using move_base_msgs::MoveBaseGoal;
using std::vector;

double get_distance(double x1, double y1, double x2, double y2)
{
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_point");
  ros::NodeHandle nh("~");
  tf::TransformListener location_listner;

  int num_points;
  double duration_time;
  double goal_tolerance;
  vector<double> x_coords, y_coords, qz_coords, qw_coords;
  nh.getParam("num_points", num_points);
  nh.getParam("points_x", x_coords);
  nh.getParam("points_y", y_coords);
  nh.getParam("points_qz", qz_coords);
  nh.getParam("points_qw", qw_coords);
  nh.getParam("point_duration", duration_time);
  nh.getParam("goal_tolerance", goal_tolerance);
  if(x_coords.size() != num_points || y_coords.size() != num_points || qz_coords.size() != num_points){
    ROS_ERROR("Wrong Point Number!");
    return 0;
  }

  vector<geometry_msgs::Pose> goal_poses;
  geometry_msgs::Pose tmp_pose;
  for(auto i = 0; i < num_points; ++i){
    tmp_pose.position.x = x_coords[i];
    tmp_pose.position.y = y_coords[i];
    tmp_pose.position.z = 0;
    tmp_pose.orientation.w = qw_coords[i];
    tmp_pose.orientation.z = qz_coords[i];
    tmp_pose.orientation.y = 0;
    tmp_pose.orientation.x = 0;
    goal_poses.push_back(tmp_pose);
  }

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  MoveBaseGoal curr_goal;
  curr_goal.target_pose.header.frame_id = "/map";

  geometry_msgs::Pose curr_pose;
  size_t point_states = 0;

  bool finish_in_time;
  tf::Quaternion q;
  while(point_states < num_points){
    curr_pose = goal_poses[point_states];
    curr_goal.target_pose.header.stamp = ros::Time::now();
    curr_goal.target_pose.pose.position.x = curr_pose.position.x;
    curr_goal.target_pose.pose.position.y = curr_pose.position.y;
    curr_goal.target_pose.pose.position.z = curr_pose.position.z;
    curr_goal.target_pose.pose.orientation.w = curr_pose.orientation.w;
    curr_goal.target_pose.pose.orientation.x = curr_pose.orientation.x;
    curr_goal.target_pose.pose.orientation.y = curr_pose.orientation.y;
    curr_goal.target_pose.pose.orientation.z = curr_pose.orientation.z;

    ac.sendGoal(curr_goal);

    finish_in_time = ac.waitForResult(ros::Duration(duration_time));
    
    // ros::Rate r(100);
    // while(ac.getState() == actionlib::SimpleClientGoalState::PENDING || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE){
    //   ROS_INFO("Approaching x: %f, y: %f, w: %f, z: %f",
    //           curr_goal.target_pose.pose.position.x,
    //           curr_goal.target_pose.pose.position.y,
    //           curr_goal.target_pose.pose.orientation.w,
    //           curr_goal.target_pose.pose.orientation.z);
    //   r.sleep();
    // }
    // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    //   ++point_states;
    //   ROS_INFO("Next Point");
    // } else {
    //   if(ac.getState() == actionlib::SimpleClientGoalState::PENDING)
    //     ROS_WARN("Pending");
    //   else if(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
    //     ROS_WARN("Preempted");
    //   else if(ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    //     ROS_WARN("Active");
    // }

    if(!finish_in_time){
      tf::StampedTransform transform;
      try {
        location_listner.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        continue;
      }

      ROS_INFO("Location: x:%f, y:%f", transform.getOrigin().x(), transform.getOrigin().y());

      if(get_distance(transform.getOrigin().x(), transform.getOrigin().y(),
                      curr_goal.target_pose.pose.position.x, curr_goal.target_pose.pose.position.y) < goal_tolerance){
        ac.cancelGoal();
        ++point_states;
        ROS_WARN("Can not reach goal in time, move to next goal");
      } else {
        ROS_WARN("Can not reach goal in time, try again");
      }
      
    } else {
      ROS_INFO("Reach goal: move to next");
      ++point_states;
    }
    if(!nh.ok()) break;
  }
  return 0;
}