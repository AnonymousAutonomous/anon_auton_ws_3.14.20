#include "ros/ros.h"
#include <obstacle_detector/Obstacles.h>
#include "std_msgs/String.h"
#include <cmath>
#include <algorithm>

ros::Publisher George;

const std::string FWD =           "cXstomf055f055";
const std::string FAST_FWD =      "cXstomf070f070";
const std::string BWD =           "cXstomr055r055";
const std::string FAST_BWD =      "cXstomr070r070";
const std::string PIVOTR =        "cXstomf060r060";
const std::string PIVOTL =        "cXstomr060f060";
const std::string FAST_PIVOTR   = "cXstomf100r100";
const std::string FAST_PIVOTL   = "cXstomr100f100";
const std::string VEER_R	= "cXstomf070f055";
const std::string VEER_L	= "cXstomf055f070";

void dirCallback(const obstacle_detector::Obstacles::ConstPtr& obs) {
  std_msgs::String msg;

  bool stop = false;
  //bool speed = false;

  bool at_wall = false;
  std::pair<double, double> closest_midpt = { 999, 999 };
  std::pair<double, double> close_wall = { 999, 999 };

  std::stringstream ss;
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

    if (pow(x,2) + pow(y,2) < 2.25) {
      at_wall = true; //If nearest point of segment obstacle is within stop radius, then the robot has readched a wall
      if (sqrt(x * x + y * y) < sqrt(close_wall.first * close_wall.first + close_wall.second * close_wall.second)) {
        close_wall.first = x;
	close_wall.second = y;
      }
    }
    /*
    if ((y > abs(x) || y < -1 * abs(x)) && pow(x,2) + pow(y,2) < 4) {
      speed = true; //Speed. I am speed.
    }
    */

    std::pair<double, double> midpt;
    midpt.first = (p1.first + p2.first) / 2.0;
    midpt.second = (p1.second + p2.second) / 2.0;

    if (sqrt(midpt.first * midpt.first + midpt.second * midpt.second) < sqrt(closest_midpt.first * closest_midpt.first + closest_midpt.second * closest_midpt.second)) {
      closest_midpt = midpt;
    }

  }

  for (const auto& circ : obs->circles) {
    double x = circ.center.x;
    double y = circ.center.y;
    if (-1 * x > abs(y) && pow(x,2) + pow(y,2) < 2.25) {
      stop = true;
    }
    /*
    if ((y > abs(x) || y < -1 * abs(x)) && pow(x,2) + pow(y,2) < 4) {
      speed = true; //Speed. I am speed.
    }
    */
  }
  
  if (stop) ss << "cAstomf000f000"; //stop
  //else if (speed) ss << "cCstomf150f150"; //ffwd --> Fee is changing the spook speed to a higher speed

  else if (at_wall) {
    if (close_wall.first < -0.5 && close_wall.second > 0) {
      ss << PIVOTL; //rotate CCW
    }
    else if (close_wall.first < -0.5 && close_wall.second < 0) {
      ss << PIVOTR; //rotate CW
    }
    else if (close_wall.first > 0.5 && close_wall.second > 0) {
      ss << PIVOTR; //rotate CW
    }
    else if (close_wall.first > 0.5 && close_wall.second < 0) {
      ss << PIVOTL; //rotate CCW
    }
    else if (close_wall.second > 1) {
      ss << VEER_L; //veer left
    }
    else if (close_wall.second < -1) {
      ss << VEER_R; //veer right
    }
    else if (close_wall.second > 0 && close_wall.second < 0.5) {
      ss << VEER_R; //veer right
    }
    else if (close_wall.second < 0 && close_wall.second > -0.5) {
      ss << VEER_L; //veer left
    }
    else {
      ss << FWD;
    }
  }

  else {
    if (closest_midpt.first > 4 * abs(closest_midpt.second)) {
      ss << FWD;
    }
    else if (closest_midpt.second > 0) {
      ss << PIVOTL; //CCW
    }
    else {
      ss << PIVOTR; //CW
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
