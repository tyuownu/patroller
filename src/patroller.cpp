#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Trigger.h"
#include "move_base_msgs/MoveBaseAction.h"

#include "vector"
#include <boost/lexical_cast.hpp>

std::vector<geometry_msgs::Point> landmarks;
visualization_msgs::MarkerArray marker_array;
ros::Publisher marker_pub;
ros::Publisher goal_publisher;
static int current_index = 0;
static bool start_patrol = false;

void clickCallBack(const geometry_msgs::PointStamped::ConstPtr& point) {
  static int i = 0;
  //marker_array.markers.resize(2*i + 2);
  geometry_msgs::Point p;
  p.x = point->point.x;
  p.y = point->point.y;
  ROS_INFO_STREAM("Adding landmark #" << i+1 << ": ( "
      << p.x << ", " << p.y << ");");

  landmarks.push_back(p);
  visualization_msgs::Marker cylinder;

  {
    cylinder.header.frame_id = "/map";
    cylinder.header.stamp = ros::Time::now();
    cylinder.ns = "cylinder";
    cylinder.id = 2*i;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.pose.orientation.w = 1.0;
    cylinder.scale.x = 0.5;
    cylinder.scale.y = 0.5;
    cylinder.scale.z = 0.01;

    cylinder.color.g = 1.0f;
    cylinder.color.a = 0.4f;
    cylinder.lifetime = ros::Duration();

    cylinder.pose.position.x = landmarks[i].x;
    cylinder.pose.position.y = landmarks[i].y;
  }
  marker_array.markers.push_back(cylinder);

  visualization_msgs::Marker text;

  {
    text.header.frame_id = "/map";
    text.header.stamp = ros::Time::now();
    text.ns = "text";
    text.id = 2*i + 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.scale.x = 1;
    text.scale.y = 1;
    text.scale.z = 1;

    text.color.r = 1.0f;
    text.color.a = 1.0f;
    text.lifetime = ros::Duration();

    text.pose.position.x = landmarks[i].x + 0.2;
    text.pose.position.y = landmarks[i].y + 0.2;

    text.text = boost::lexical_cast<std::string>(i+1);
  }
  marker_array.markers.push_back(text);
  i++;
  
  marker_pub.publish(marker_array);
}

void publishNextGoal(bool first_input) {

  if ( !first_input )
  current_index++;
  if ( current_index >= landmarks.size() )
    current_index = 0;
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";
  goal.pose.position.x = landmarks[current_index].x;
  goal.pose.position.y = landmarks[current_index].y;
  goal.pose.orientation.w = 1.0f;

  goal_publisher.publish(goal);
}

void positionCallback(
    const move_base_msgs::MoveBaseActionFeedback::ConstPtr& position) {
  if ( !start_patrol ) return;
  if ( (position->feedback.base_position.pose.position.x -landmarks[current_index].x) *
      (position->feedback.base_position.pose.position.x - landmarks[current_index].x) +
      (position->feedback.base_position.pose.position.y -landmarks[current_index].y) *
      (position->feedback.base_position.pose.position.y - landmarks[current_index].y) < 0.09 ) { // 0.3m
    publishNextGoal(false);
  }
}

bool receiveStartCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {

  if ( landmarks.size() < 2 ) {
    ROS_ERROR("The landmark number is not correct!");
    res.success = false;
    res.message = "There are at lest have 2 landmarks.";
    return false;
  } else {
    start_patrol = true;
    res.success = true;
    res.message = "Start patrol.";
    publishNextGoal(true);
    return true;
  }
}

bool receiveStopCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  start_patrol = false;
  res.success = true;
  res.message = "Stop patrol.";
  return true;
}

bool receiveResetCommand(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  start_patrol = false;
  res.success = true;
  res.message = "Reset patrol.";
  return true;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"patroller");
    ros::NodeHandle n;

    //ros::Rate r(1);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("landmarks",10);
    goal_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);
    ros::Subscriber click_sub =
      n.subscribe("/clicked_point", 10, clickCallBack);
    ros::Subscriber current_position_sub =
      n.subscribe("/move_base/feedback", 10, positionCallback);

    ros::ServiceServer cmd_start_server =
      n.advertiseService("StartPatrol", &receiveStartCommand);
    ros::ServiceServer cmd_stop_server =
      n.advertiseService("StopPatrol", &receiveStopCommand);
    ros::ServiceServer cmd_reset_server =
      n.advertiseService("ResetPatrol", &receiveResetCommand);

    ros::spin();
    return 0;
}
