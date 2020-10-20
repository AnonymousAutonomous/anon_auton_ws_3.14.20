// OBSOLETE - DO NOT USE

#include "ros/ros.h"
#include <obstacle_detector/Obstacles.h>
#include "std_msgs/Float64.h"
#include <cmath>
#include <algorithm>

ros::Publisher George;
std::pair<double, double> prev_line = { 0, 0 };
double r = 12;

double multiplier = 10;

void dirCallback(const obstacle_detector::Obstacles::ConstPtr& obs) {
  std_msgs::Float64 msg;

  std::pair<double, double> closest_line;
  double distance = 9999;

  std::stringstream ss;
  for (const auto& seg : obs->segments) {
    std::pair<double, double> p1;
    std::pair<double, double> p2;
    p1.first = seg.first_point.x;
    p1.second = seg.first_point.y;
    p2.first = seg.last_point.x;
    p2.second = seg.last_point.y;

    if (p1.first < -1 * abs(p1.second)) {
      continue;
    }
    if (p2.first < -1 * abs(p2.second)) {
      continue;
    }

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

    double x = p3.first;
    double y = p3.second;

    if (sqrt(x * x + y * y) < distance) {
    	distance = sqrt(x * x + y * y);
	closest_line = line1;
    }
  }

  std::pair<double, double> cpa;
  std::pair<double, double> cpb;

  double A = -1 * closest_line.first;
  double B = 1;
  double C = -1 * closest_line.second;

  double x0 = -1 * A * C / (A * A + B * B);
  double y0 = -1 * B * C / (A * A + B * B);
  double d = sqrt(r * r - C * C / (A * A + B * B));
  double m = sqrt(d * d / (A * A + B * B));
  
  cpa.first = x0 + B * m; cpa.second = y0 - A * m;
  cpb.first = x0 - B * m; cpb.second = y0 + A * m;

  std::pair<double, double> ppa;
  std::pair<double, double> ppb;

  A = -1 * prev_line.first;
  B = 1;
  C = -1 * prev_line.second;

  x0 = -1 * A * C / (A * A + B * B);
  y0 = -1 * B * C / (A * A + B * B);
  d = sqrt(r * r - C * C / (A * A + B * B));
  m = sqrt(d * d / (A * A + B * B));
  
  ppa.first = x0 + B * m; ppa.second = y0 - A * m;
  ppb.first = x0 - B * m; ppb.second = y0 + A * m;

  double fresh_a = sqrt((cpa.first - ppa.first) * (cpa.first - ppa.first)
		  + (cpa.second - ppa.second) * (cpa.second - ppa.second));
  double fresh_b = sqrt((cpb.first - ppb.first) * (cpb.first - ppb.first)
		  + (cpb.second - ppb.second) * (cpb.second - ppb.second));

  double frechet;
  if (fresh_a < fresh_b) frechet = fresh_b;
  else frechet = fresh_a;

  frechet *= multiplier;

  prev_line = closest_line;

  msg.data = frechet;

  George.publish(msg);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "frechet");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("raw_obstacles", 10, dirCallback);

  ros::NodeHandle oi;
  George = oi.advertise<std_msgs::Float64>("fresh", 1000);

  ros::spin();

  return 0;
}
