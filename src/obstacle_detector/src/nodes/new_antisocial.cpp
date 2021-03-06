#include "ros/ros.h"
#include <obstacle_detector/Obstacles.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "../../../../constants/tarts.h"
#include "../../../../constants/choreos.h"
#include <cmath>
#include <algorithm>

Tart antisocial_standard(
  {
    {3*M_PI/4, 5*M_PI/4, 0.5} // 0
  },
  {
    {M_PI/2 - 0.1, M_PI/2 + 0.1, 0.5}, // 0
    {3*M_PI/2 - 0.1, 3*M_PI/2 + 0.1, 0.5}, // 1
    {M_PI/2 - 0.1, M_PI/2 + 0.1, 1}, // 2
    {3*M_PI/2 - 0.1, 3*M_PI/2 + 0.1, 1}, // 3
    {M_PI/2 - 0.1, M_PI/2 + 0.1, 1.5}, // 4
    {3*M_PI/2 - 0.1, 3*M_PI/2 + 0.1, 1.5}, // 5
    {0, M_PI/2 - 0.1, 0.5}, // 6
    {M_PI/2 + 0.1, M_PI, 0.5}, // 7
    {M_PI, 3*M_PI/2 - 0.1, 0.5}, // 8
    {3*M_PI/2 + 0.1, 2*M_PI, 0.5}, // 9
    {0, M_PI/2 - 0.1, 1}, // 10
    {M_PI/2 + 0.1, M_PI, 1}, // 11
    {M_PI, 3*M_PI/2 - 0.1, 1}, // 12
    {3*M_PI/2 + 0.1, 2*M_PI, 1}, // 13
    {0, M_PI/2 - 0.1, 1.5}, // 14
    {M_PI/2 + 0.1, M_PI, 1.5}, // 15
    {M_PI, 3*M_PI/2 - 0.1, 1.5}, // 16
    {3*M_PI/2 + 0.1, 2*M_PI, 1.5} // 17
  },
  {
    {{{0},{}}, STOP},
    {{{},{6}}, PIVOTR},
    {{{},{7}}, PIVOTL},
    {{{},{8}}, PIVOTR},
    {{{},{9}}, PIVOTL},
    {{{},{0}}, FWD},
    {{{},{1}}, FWD},
    {{{},{10}}, PIVOTR},
    {{{},{11}}, VEERL},
    {{{},{12}}, VEERR},
    {{{},{13}}, PIVOTL},
    {{{},{2}}, FWD},
    {{{},{3}}, FWD},
    {{{},{14}}, PIVOTR},
    {{{},{15}}, SLIGHTL},
    {{{},{16}}, SLIGHTR},
    {{{},{17}}, PIVOTL},
    {{{},{4}}, FWD},
    {{{},{5}}, FWD}
  }
);

ros::Publisher George;

void dirCallback(const obstacle_detector::Obstacles::ConstPtr& obs) {
  std_msgs::String msg;

  bool at_wall = false;
  std::pair<double, double> closest_midpt = { 999, 999 };
  std::pair<double, double> close_wall = { 999, 999 };
  // sets closest midpoint and closest wall to absurdly large numbers
  // guarantees overwrite upon first read in

  std::stringstream ss;
  for (const auto& seg : obs->segments) {
    std::pair<double, double> p1;
    std::pair<double, double> p2;
    p1.first = seg.first_point.x;
    p1.second = seg.first_point.y;
    p2.first = seg.last_point.x;
    p2.second = seg.last_point.y;
    // for each segment, store endpoints as p1 and p2

    std::pair<double, double> line1;
    line1.first = (p2.second - p1.second) / (p2.first - p1.first);
    line1.second = p1.second - line1.first * p1.first;
    // line1 is slope and intercept of segment of interest

    std::pair<double, double> line2;
    line2.first = -1. / line1.first;
    line2.second = 0.;
    // line2 is perpendicular to line1 and through origin

    double determinant = (-1) * line1.first * 1 - (-1) * line2.first * 1;
    
    std::pair<double, double> p3;
    p3.first = (line1.second * 1 - line2.second * 1) / determinant;
    p3.second = (-1 * line1.first * line2.second - (-1) * line2.first * line1.second) / determinant;
    // p3 is the intersection of line1 and line2

    double x;
    double y;

    if (std::min(p1.first, p2.first) <= p3.first &&
	p3.first <= std::max(p1.first, p2.first) &&
	std::min(p1.second, p2.second) <= p3.second &&
	p3.second <= std::max(p1.second, p2.second)) {
      x = p3.first;
      y = p3.second;//use p3, p3 is within the line segment
    }
    else {
      double dist_cf = sqrt(pow((p1.first - p3.first),2) + pow((p1.second - p3.second),2));
      double dist_cl = sqrt(pow((p2.first - p3.first),2) + pow((p2.second - p3.second),2));
      if (dist_cf < dist_cl) {
        x = p1.first;
	y = p1.second;//use p1, closest point on line segment
      }
      else {
        x = p2.first;
	y = p2.second;//use p2, closest point on line segment
      }
    }

    if (sqrt(x*x + y*y) < sqrt(close_wall.first*close_wall.first + close_wall.second*close_wall.second)) {
      close_wall.first = x;
      close_wall.second = y;
    }

    std::pair<double, double> midpt;
    midpt.first = (p1.first + p2.first) / 2.0;
    midpt.second = (p1.second + p2.second) / 2.0;

    if (sqrt(midpt.first*midpt.first + midpt.second*midpt.second) < sqrt(closest_midpt.first*closest_midpt.first + closest_midpt.second*closest_midpt.second)) {
      closest_midpt = midpt;
    }
  }

  antisocial_standard.line_update(close_wall.first, close_wall.second);

  for (const auto& circ : obs->circles) {
    double x = circ.center.x;
    double y = circ.center.y;
    antisocial_standard.circle_update(x, y);
  }

  std::string result = antisocial_standard.evaluate();
  if (result == "") {
    ROS_INFO("FINDING WALL...");
    if (closest_midpt.first < -4 * abs(closest_midpt.second)) {
      ss << FWD;
    }
    else if (closest_midpt.second > 0) {
      ss << PIVOTR; //CW
    }
    else {
      ss << PIVOTL; //CCW
    }
  }
  else {
    ss << result;
  }

  msg.data = ss.str();
  George.publish(msg);
  antisocial_standard.reset();
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "antisocial");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("raw_obstacles", 10, dirCallback);

  ros::NodeHandle oi;
  George = oi.advertise<std_msgs::String>("larry", 1000);

  ros::spin();

  return 0;
}
