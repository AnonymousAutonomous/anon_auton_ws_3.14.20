#include "ros/ros.h"
#include <obstacle_detector/Obstacles.h>
#include "std_msgs/String.h"
#include <cmath>
#include <algorithm>

ros::Publisher George;

const std::string FWD =           "cXstomf065f065";
const std::string FAST_FWD =      "cXstomf080f080";
const std::string BWD =           "cXstomr065r065";
const std::string FAST_BWD =      "cXstomr080r080";
const std::string PIVOTR =        "cXstomf070r070";
const std::string PIVOTL =        "cXstomr070f070";
const std::string FAST_PIVOTR   = "cXstomf110r110";
const std::string FAST_PIVOTL   = "cXstomr110f110";
const std::string VEER_R        = "cXstomf160f065";
const std::string VEER_L        = "cXstomf065f160";

void dirCallback(const obstacle_detector::Obstacles::ConstPtr& obs) {
  std_msgs::String msg;

  bool stop = false;
  //bool speed = false;
  
  bool by_agent = false;
  std::pair<double, double> closest_agent = { -999, 0 };

  std::stringstream ss;
  for (const auto& circ : obs->circles) {
    double x = circ.center.x;
    double y = circ.center.y;
    if (pow(x,2) + pow(y,2) < 1 && x < abs(y)) { // changed from 2.25 to 1
      ROS_INFO("%f,%f",x,y);	    
      by_agent = true;
    }
    else {
      if (sqrt(x * x + y * y) < sqrt(closest_agent.first * closest_agent.first + closest_agent.second * closest_agent.second) && x < abs(y)) {
        closest_agent.first = x;
	closest_agent.second = y;
      }
    }
    /*
    if ((y > abs(x) || y < -1 * abs(x)) && pow(x,2) + pow(y,2) < 4) {
      speed = true; //Speed. I am speed.
    }
    */
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

    if (-1 * x > abs(y) && pow(x,2) + pow(y,2) < 1) { // changed from 2.25 to 1
      stop = true;
    }
    /*
    if ((y > abs(x) || y < -1 * abs(x)) && pow(x,2) + pow(y,2) < 4) {
      speed = true; //Speed. I am speed.
    }
    */
  }

  if (stop) {
    ss << "cAstomf000f000"; //stop
    ROS_INFO("STOPPED");
  }
  //else if (speed) ss << "cCstomf150f150"; //ffwd --> Fee is changing the spook speed to a higher speed
  
  else if (by_agent) {
    ss << "cAstomf000f000"; //dance
    ROS_INFO("BY AGENT");
  }
  else {
    ROS_INFO("%f,%f", closest_agent.first, closest_agent.second);
    if (closest_agent.first < -4 * abs(closest_agent.second)) {
      ss << FWD;
      ROS_INFO("APPROACHING AGENT");
    }
    else if (closest_agent.second > 0) {
      ss << PIVOTR; //CW
      ROS_INFO("TURNING CLOCKWISE TOWARD AGENT");
    }
    else {
      ss << PIVOTL; //CCW
      ROS_INFO("TURNING COUNTER CLOCKWISE TOWARD AGENT");
    }
  }

  msg.data = ss.str();

  George.publish(msg);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "director");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("raw_obstacles", 10, dirCallback);

  ros::NodeHandle oi;
  George = oi.advertise<std_msgs::String>("larry", 1000);

  ros::spin();

  return 0;
}
