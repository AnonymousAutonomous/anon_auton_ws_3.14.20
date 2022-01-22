#include "ros/ros.h"
#include <obstacle_detector/Obstacles.h>
#include "std_msgs/String.h"
#include "../../../../constants/str_cmds.h"
#include "../../../../constants/pies.h"
#include <cmath>
#include <algorithm>

// All available autonomous commands
std::map<std::string, std::string> commands_in;
std::unordered_map<AutonomousCmd, std::string> commands;

Pie new_director_standard(
        {
                {3*M_PI/4, 5*M_PI/4, 0.5},
                {M_PI/4, 3*M_PI/4, 1},
                {5*M_PI/4, 7*M_PI/4, 1}
        },
        {
                {{0}, commands[STOP]},
                {{1}, commands[SPOOK]},
                {{2}, commands[SPOOK]}
        }
);

ros::Publisher George;

void dirCallback(const obstacle_detector::Obstacles::ConstPtr& obs) {
  std_msgs::String msg;

  bool stop = false;
  bool speed = false;

  std::stringstream ss;
  for (const auto& circ : obs->circles) {
    double x = circ.center.x;
    double y = circ.center.y;
    new_director_standard.update(x,y);
  }
  for (const auto& seg : obs->segments) {
    std::pair<double, double> p1;
    std::pair<double, double> p2;
    p1.first = seg.first_point.x;
    p1.second = seg.first_point.y;
    p2.first = seg.last_point.x;
    p2.second = seg.last_point.y;

    std::pair<double, double> line1;
    line1.first = (p2.second - p1.second) / (p2.first - p1.first);
    line1.second = p1.second - line1.first * p1.first;

    std::pair<double, double> line2;
    line2.first = -1. / line1.first;
    line2.second = 0.;

    double determinant = (-1) * line1.first * 1 - (-1) * line2.first * 1;
    
    std::pair<double, double> p3;
    p3.first = (line1.second * 1 - line2.second * 1) / determinant;
    p3.second = (-1 * line1.first * line2.second - (-1) * line2.first * line1.second) / determinant;

    double x;
    double y;

    if (std::min(p1.first, p2.first) <= p3.first &&
	p3.first <= std::max(p1.first, p2.first) &&
	std::min(p1.second, p2.second) <= p3.second &&
	p3.second <= std::max(p1.second, p2.second)) {
      x = p3.first;
      y = p3.second;//use p3
    }
    else {
      double dist_cf = sqrt(pow((p1.first - p3.first),2) + pow((p1.second - p3.second),2));
      double dist_cl = sqrt(pow((p2.first - p3.first),2) + pow((p2.second - p3.second),2));
      if (dist_cf < dist_cl) {
        x = p1.first;
	y = p1.second;//use p1
      }
      else {
        x = p2.first;
	y = p2.second;//use p2
      }
    }
    new_director_standard.update(x,y);
  }

  ss << new_director_standard.evaluate();

  msg.data = ss.str();

  George.publish(msg);
  new_director_standard.reset();
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "director");
  ros::NodeHandle nh;
  if (nh.getParam("/autonomous", commands_in)) {
        for (auto i = commands_in.begin(); i != commands_in.end(); i++) {
            commands[AUTOCMD_STRING_TO_ENUM[i->first]] = i->second;
        }
        ROS_INFO("Autonomous commands have been loaded for new director.");
    }
    else {
        ROS_INFO("You must load autonomous commands before using new director.");
        return 1;
    }
  ros::Subscriber sub = nh.subscribe("raw_obstacles", 10, dirCallback);

  ros::NodeHandle oi;
  George = oi.advertise<std_msgs::String>("larry", 1000);

  ros::spin();

  return 0;
}
