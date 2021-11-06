// nice

#include "ros/ros.h"
#include <obstacle_detector/Obstacles.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "../../../../constants/tarts.h"
#include "../../../../constants/choreos.h"
#include <cmath>
#include <algorithm>

int stop_counter = 0;
int stop_limit = 150;

int move_counter = 0;
int move_limit = stop_limit / 5;

bool listening = true;

enum class state : char {wait, noise, pivot, noise_again, pivot_again, escape};
state mode = state::wait;

std::string high_priority(std::string str) {
	str[0] = '0';
	return str;
}

Tart standard(
        {
                {3*M_PI/4, 5*M_PI/4, 0.5},
                {M_PI/4, 3*M_PI/4, 1},
                {5*M_PI/4, 7*M_PI/4, 1}
        },
	{
		{3*M_PI/4, 5*M_PI/4, 0.5},
		{M_PI/4, 3*M_PI/4, 1},
		{5*M_PI/4, 7*M_PI/4, 1}
	},
        {
		// {{{0,1},{}}, "CIRCLE FRONT, CIRCLE RIGHT"},
		// {{{0},{1}}, "CIRCLE FRONT, LINE RIGHT"},
                {{{0},{}}, STOP},
		{{{},{0}}, STOP},
		{{{1},{}}, SPOOK},
		{{{2},{}}, SPOOK},
                {{{},{1}}, SPOOK},
                {{{},{2}}, SPOOK}
        }
);

// Escape is used in two cases
// - chair is encircled by people
// - chair is encircled by paper
// In both cases, chair should use LIDAR to navigate and escape
// Goal is constant forward progress
// - chair does not stop
// - chair turns away from obstacles
// - chair tries to find a clear path forward
// All commands should be high priority
// Two cases may require separate tarts in the future
Tart escape(
	{
		{3*M_PI/4, 5*M_PI/4, 0.5},
		{M_PI/4, 3*M_PI/4, 0.5},
		{5*M_PI/4, 7*M_PI/4, 0.5}
	},
	{
		{3*M_PI/4, 5*M_PI/4, 0.5},
		{M_PI/4, 3*M_PI/4, 0.5},
		{5*M_PI/4, 7*M_PI/4, 0.5}
	},
	{
		{{{0,1,2},{}}, high_priority(PIVOTR)},
		{{{},{0,1,2}}, high_priority(PIVOTR)},
		{{{0,1},{}}, high_priority(PIVOTL)},
		{{{},{0,1}}, high_priority(PIVOTL)},
		{{{0,2},{}}, high_priority(PIVOTR)},
		{{{},{0,2}}, high_priority(PIVOTR)},
		{{{0},{}}, high_priority(PIVOTR)},
		{{{},{0}}, high_priority(PIVOTR)},
		{{{1,2},{}}, high_priority(FWD)},
		{{{},{1,2}}, high_priority(FWD)},
		{{{1},{}}, high_priority(VEERL)},
		{{{},{1}}, high_priority(VEERL)},
		{{{2},{}}, high_priority(VEERR)},
		{{{},{2}}, high_priority(VEERR)},
		{{{},{}}, high_priority(FWD)}
	}
);

Tart& tart_in_use = standard;

ros::Publisher George;
ros::Publisher audio_pub;

void pauseCallback(const std_msgs::Empty empty_msg) {
  listening = true;
}

void dirCallback(const obstacle_detector::Obstacles::ConstPtr& obs) {
  if (!listening) return;

  std_msgs::String msg;

  bool stop = false;
  bool speed = false;

  std::stringstream ss;
  for (const auto& circ : obs->circles) {
    double x = circ.center.x;
    double y = circ.center.y;
    tart_in_use.circle_update(x,y);
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
      y = p3.second; //use p3
    }
    else {
      double dist_cf = sqrt(pow((p1.first - p3.first),2) + pow((p1.second - p3.second),2));
      double dist_cl = sqrt(pow((p2.first - p3.first),2) + pow((p2.second - p3.second),2));
      if (dist_cf < dist_cl) {
        x = p1.first;
	y = p1.second; //use p1
      }
      else {
        x = p2.first;
	y = p2.second; //use p2
      }
    }
    tart_in_use.line_update(x,y);
  }

  ss << tart_in_use.evaluate();

  msg.data = ss.str();

  if (msg.data == STOP) {
  	++stop_counter;
  }
  else {
  	++move_counter;
  }
  if (stop_counter >= stop_limit) {
  	stop_counter = 0;
	move_counter = 0;
	// Check state of chair
	switch (mode) {
		case state::wait:
		{
			std_msgs::String audio_msg;
			audio_msg.data = "beep";
			audio_pub.publish(audio_msg);
			mode = state::noise;
			break;
		}
		case state::noise:
		{
			msg.data = RREVERSE;
			mode = state::pivot;
			break;
		}
		case state::pivot:
		{
			std_msgs::String audio_msg;
			audio_msg.data = "beep";
			audio_pub.publish(audio_msg);
			audio_pub.publish(audio_msg);
			mode = state::noise_again;
			break;
		}
		case state::noise_again:
		{
			msg.data = RREVERSE;
			mode = state::pivot_again;
			break;
		}
		case state::pivot_again:
		{
			// Change tarts
			tart_in_use = escape;
			stop_counter = stop_limit; // force chair to stay in FSM
			mode = state::escape;
			// TODO: record initial time
			break;
		}
		case state::escape:
		{
			// TODO: after some time, change tarts back
			stop_counter = stop_limit; // force chair to stay in FSM
			mode = state::wait;
			break;
		}
		default:
		{
			ROS_INFO("How did we get here? LIDAR edition");
			break;
		}
	}
  }
  else if (move_counter >= move_limit) {
  	stop_counter = 0;
	move_counter = 0;
	mode = state::wait;
  }

  if (msg.data[1] == 'C') {
	listening = false;
  }

  George.publish(msg);
  tart_in_use.reset();
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "multi_tart");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("raw_obstacles", 10, dirCallback);
  ros::Subscriber eoc_sub = nh.subscribe("end_of_choreo", 1000, pauseCallback);

  ros::NodeHandle oi;
  George = oi.advertise<std_msgs::String>("larry", 1000);
  audio_pub = oi.advertise<std_msgs::String>("audio_channel", 1000);

  ros::waitForShutdown();

  return 0;
}
