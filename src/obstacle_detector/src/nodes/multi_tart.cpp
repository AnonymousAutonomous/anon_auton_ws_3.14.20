// nice time skew, lol, oops

#include "ros/ros.h"
#include <obstacle_detector/Obstacles.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Char.h"
#include "std_msgs/UInt8.h"
#include "../../../../constants/str_cmds.h"
#include "../../../../constants/tarts.h"
#include "../../../../constants/choreos.h"
#include <cmath>
#include <algorithm>

// All available autonomous commands
std::map<std::string, std::string> commands_in;
std::unordered_map<AutonomousCmd, std::string> commands;


int stop_counter = 0;
int stop_limit = 150;

int move_counter = 0;
int move_limit = stop_limit / 5;

int turn_counter = 0;
int turn_limit = 100;

int proc_counter = 0;
int proc_limit = turn_limit / 5;

int escape_counter = 0;
int escape_limit = 500;

bool listening = true;

char queue_state = 'A';
uint8_t camera_state = 0;

enum class state : char {
  wait,
  // LIDAR protocol
  noise,
  pivot,
  noise_again,
  pivot_again,
  escape,
  // CAMERA protocol
  turn,
  help
  // TODO: add beacon state?
};

// mirror chair status enums in hub manager
enum class chair_broadcast_status : char {ready, exclude, success, failure};
enum class chair_stuck_status : char {stuck, not_stuck};
enum class chair_trapped_status : char {trapped, not_trapped};

state mode = state::wait;

std::string high_priority(std::string str) {
	str[0] = '0';
	return str;
}

Tart standard(
        {
                {3*M_PI/4, 5*M_PI/4, 1},
                {M_PI/4, 3*M_PI/4, 1},
                {5*M_PI/4, 7*M_PI/4, 1}
        },
	{
		{3*M_PI/4, 5*M_PI/4, 1},
		{M_PI/4, 3*M_PI/4, 1},
		{5*M_PI/4, 7*M_PI/4, 1}
	},
        {
		// {{{0,1},{}}, "CIRCLE FRONT, CIRCLE RIGHT"},
		// {{{0},{1}}, "CIRCLE FRONT, LINE RIGHT"},
                {{{0},{}}, commands[STOP]},
		{{{},{0}}, commands[STOP]},
		{{{1},{}}, commands[SPOOK]},
		{{{2},{}}, commands[SPOOK]},
                {{{},{1}}, commands[SPOOK]},
                {{{},{2}}, commands[SPOOK]}
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
		{3*M_PI/4, 5*M_PI/4, 1},
		{M_PI/4, 3*M_PI/4, 1},
		{5*M_PI/4, 7*M_PI/4, 1}
	},
	{
		{3*M_PI/4, 5*M_PI/4, 1},
		{M_PI/4, 3*M_PI/4, 1},
		{5*M_PI/4, 7*M_PI/4, 1}
	},
	{
		{{{0,1,2},{}}, commands[PIVOTR]},
		{{{},{0,1,2}}, commands[PIVOTR]},
		{{{0,1},{}}, commands[PIVOTL]},
		{{{},{0,1}}, commands[PIVOTL]},
		{{{0,2},{}}, commands[PIVOTR]},
		{{{},{0,2}}, commands[PIVOTR]},
		{{{0},{}}, commands[PIVOTR]},
		{{{},{0}}, commands[PIVOTR]},
		{{{1,2},{}}, commands[FWD]},
		{{{},{1,2}}, commands[FWD]},
		{{{1},{}}, commands[VEERL]},
		{{{},{1}}, commands[VEERL]},
		{{{2},{}}, commands[VEERR]},
		{{{},{2}}, commands[VEERR]},
		{{{},{}}, commands[FWD]}
	}
);

Tart* tart_in_use = &standard;

ros::Publisher George;
ros::Publisher audio_pub;
ros::Publisher update_hub_pub;

void reset_lidar_counters() {
  stop_counter = 0;
  move_counter = 0;
}

void reset_camera_counters() {
  turn_counter = 0;
  proc_counter = 0;
}

void reset_escape_counters() {
  escape_counter = 0;
}

void reset_all_counters() {
  reset_lidar_counters();
  reset_camera_counters();
  reset_escape_counters();
}

void queueCallback(const std_msgs::Char char_msg) {
  queue_state = char_msg.data;
}

void cameraCallback(const std_msgs::UInt8 uint8_msg) {
  camera_state = uint8_msg.data;
}

void pauseCallback(const std_msgs::Empty empty_msg) {
  queue_state = 'A';
  listening = true;
}

void dirCallback(const obstacle_detector::Obstacles::ConstPtr& obs) {
  // if (!listening) return;
  // can restore listening functionality if necessary

  std_msgs::String msg;

  std::stringstream ss;
  for (const auto& circ : obs->circles) {
    double x = circ.center.x;
    double y = circ.center.y;
    tart_in_use->circle_update(x,y);
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
    tart_in_use->line_update(x,y);
  }

  ss << tart_in_use->evaluate();

  msg.data = ss.str();

  switch (mode) {
    case state::wait:
    {
      switch (queue_state) {
        case 'A':
	{
	  msg.data == commands[STOP] ? ++stop_counter : ++move_counter;
	  camera_state >= 4 ? ++turn_counter : ++proc_counter;
	  // camera trans
	  // LIDAR trans
	  if (turn_counter >= turn_limit) {
	    mode = state::turn;
	    ROS_INFO("GOING TO TURN");
	    reset_lidar_counters();
	    reset_camera_counters();
	  }
	  else if (stop_counter >= stop_limit) {
	    std_msgs::String audio_msg;
	    audio_msg.data = "beep";
	    audio_pub.publish(audio_msg);
	    mode = state::noise;
	    ROS_INFO("GOING TO NOISE");
	    reset_lidar_counters();
	    reset_camera_counters();
	  }
	  else {
	    if (move_counter >= move_limit) {
	      mode = state::wait;
	      reset_lidar_counters();
	    }
	    if (proc_counter >= proc_limit) {
	      mode = state::wait;
	      reset_camera_counters();
	    }
	  }
	  break;
	}
	case 'C':
	{
	  reset_lidar_counters();
	  camera_state >= 4 ? ++turn_counter : ++proc_counter;
	  // camera trans
	  if (turn_counter >= turn_limit) {
	    mode = state::turn;
	    ROS_INFO("GOING TO TURN");
	    reset_camera_counters();
	  }
	  else if (proc_counter >= proc_limit) {
	    mode = state::wait;
	    reset_camera_counters();
	  }
	  break;
	}
	default:
	{
	  reset_all_counters();
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    case state::noise:
    {
      switch (queue_state) {
        case 'A':
	{
	  msg.data == commands[STOP] ? ++stop_counter : ++move_counter;
	  reset_camera_counters();
	  // LIDAR trans
	  if (stop_counter >= stop_limit) {
	    msg.data = commands[RREVERSE];
	    mode = state::pivot;
	    ROS_INFO("GOING TO PIVOT");
	    reset_lidar_counters();
	  }
	  else if (move_counter >= move_limit) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	case 'C':
	{
	  reset_camera_counters();
	  if (listening) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	default:
	{
	  reset_all_counters();
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    case state::pivot:
    {
      switch (queue_state) {
        case 'A':
	{
	  msg.data == commands[STOP] ? ++stop_counter : ++move_counter;
	  reset_camera_counters();
	  // LIDAR trans
	  if (stop_counter >= stop_limit) {
	    std_msgs::String audio_msg;
	    audio_msg.data = "beep";
	    audio_pub.publish(audio_msg);
	    audio_pub.publish(audio_msg);
	    mode = state::noise_again;
	    ROS_INFO("GOING TO NOISE AGAIN");
	    reset_lidar_counters();
	  }
	  else if (move_counter >= move_limit) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	case 'C':
	{
	  reset_camera_counters();
	  if (listening) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	default:
	{
	  reset_all_counters();
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    case state::noise_again:
    {
      switch (queue_state) {
        case 'A':
	{
	  msg.data == commands[STOP] ? ++stop_counter : ++move_counter;
	  reset_camera_counters();
	  // LIDAR trans
	  if (stop_counter >= stop_limit) {
	    msg.data = commands[RREVERSE];
	    mode = state::pivot_again;
	    ROS_INFO("GOING TO PIVOT AGAIN");
	    reset_lidar_counters();
	  }
	  else if (move_counter >= move_limit) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	case 'C':
	{
	  reset_camera_counters();
	  if (listening) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	default:
	{
	  reset_all_counters();
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    case state::pivot_again:
    {
      switch (queue_state) {
        case 'A':
	{
	  msg.data == commands[STOP] ? ++stop_counter : ++move_counter;
	  reset_camera_counters();
	  // LIDAR trans
	  if (stop_counter >= stop_limit) {
	    // Change tarts
	    tart_in_use = &escape;
	    mode = state::escape;
	    ROS_INFO("GOING TO ESCAPE");
	    reset_lidar_counters();
	  }
	  else if (move_counter >= move_limit) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	case 'C':
	{
	  reset_camera_counters();
	  if (listening) {
	    mode = state::wait;
	    reset_lidar_counters();
	  }
	  break;
	}
	default:
	{
	  reset_all_counters();
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    case state::turn:
    {
      switch (queue_state) {
        case 'A':
	{
	  reset_lidar_counters();
	  camera_state >= 4 ? ++turn_counter : ++proc_counter;
	  // camera trans
	  if (turn_counter >= turn_limit) {
	    std_msgs::String to_hub;
	    to_hub.data = "0T";
	    to_hub.data.push_back(static_cast<char>(chair_trapped_status::trapped));
	    update_hub_pub.publish(to_hub);
	    mode = state::help;
	    ROS_INFO("GOING TO HELP");
	    reset_camera_counters();
	  }
	  else if (proc_counter >= proc_limit) {
	    mode = state::wait;
	    reset_camera_counters();
	  }
	  break;
	}
	case 'C':
	{
	  reset_lidar_counters();
	  camera_state >= 4 ? ++turn_counter : ++proc_counter;
	  // camera trans
	  if (turn_counter >= turn_limit) {
	    std_msgs::String to_hub;
	    to_hub.data = "0T";
	    to_hub.data.push_back(static_cast<char>(chair_trapped_status::trapped));
	    update_hub_pub.publish(to_hub);
	    mode = state::help;
	    ROS_INFO("GOING TO HELP");
	    reset_camera_counters();
	  }
	  else if (proc_counter >= proc_limit) {
	    mode = state::wait;
	    reset_camera_counters();
	  }
	  break;
	}
	default:
	{
	  reset_all_counters();
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    case state::escape:
    {
      switch (queue_state) {
        case 'A':
	{
	  reset_lidar_counters();
	  reset_camera_counters();
	  ++escape_counter;
	  if (escape_counter >= escape_limit) {
	    // Change tarts
	    tart_in_use = &standard;
	    mode = state::wait;
	    reset_escape_counters();
	  }
	  break;
	}
	case 'C':
	{
	  reset_lidar_counters();
	  reset_camera_counters();
	  ++escape_counter;
	  if (escape_counter >= escape_limit) {
	    // Change tarts
	    tart_in_use = &standard;
	    mode = state::wait;
	    reset_escape_counters();
	  }
	  break;
	}
	default:
	{
	  reset_all_counters();
	  tart_in_use = &standard;
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    case state::help:
    {
      switch (queue_state) {
        case 'A':
	{
	  reset_all_counters();
	  break;
	}
	case 'C':
	{
	  reset_all_counters();
	  break;
	}
	default:
	{
	  reset_all_counters();
	  mode = state::wait;
	  break;
	}
      }
      break;
    }
    default:
    {
      ROS_INFO("How did we get here? LIDAR edition");
      break;
    }
  }

/*
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
			ROS_INFO("GOING TO NOISE");
			break;
		}
		case state::noise:
		{
			msg.data = RREVERSE;
			mode = state::pivot;
			ROS_INFO("GOING TO PIVOT");
			break;
		}
		case state::pivot:
		{
			std_msgs::String audio_msg;
			audio_msg.data = "beep";
			audio_pub.publish(audio_msg);
			audio_pub.publish(audio_msg);
			mode = state::noise_again;
			ROS_INFO("GOING TO NOISE AGAIN");
			break;
		}
		case state::noise_again:
		{
			msg.data = RREVERSE;
			mode = state::pivot_again;
			ROS_INFO("GOING TO PIVOT AGAIN");
			break;
		}
		case state::pivot_again:
		{
			// Change tarts
			tart_in_use = &escape;
			mode = state::escape;
			ROS_INFO("GOING TO ESCAPE");
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
	if (mode == state::escape) {
		stop_counter = 0;
		move_counter = 0;
		++escape_counter;
		if (escape_counter >= escape_limit) {
			escape_counter = 0;
			tart_in_use = &standard;
			mode = state::wait;
		}
	}
	else {
  		stop_counter = 0;
		move_counter = 0;
		escape_counter = 0;
		mode = state::wait;
	}
  }
*/

  if (msg.data[1] == 'C') {
    listening = false;
  }

  George.publish(msg);
  tart_in_use->reset();

  ROS_INFO("STOP COUNTER: %d", stop_counter);
  ROS_INFO("MOVE COUNTER: %d", move_counter);
  ROS_INFO("TURN COUNTER: %d", turn_counter);
  ROS_INFO("PROC COUNTER: %d", proc_counter);
  ROS_INFO("ESCAPE COUNTER: %d", escape_counter);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "multi_tart");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::NodeHandle nh;
  if (nh.getParam("autonomous", commands_in)) {
        for (auto i = commands_in.begin(); i != commands_in.end(); i++) {
            commands[AUTOCMD_STRING_TO_ENUM[i->first]] = i->second;
        }
        ROS_INFO("Autonomous commands have been loaded for multitart.");
    }
    else {
        ROS_INFO("You must load autonomous commands before using multitart.");
        return 1;
    }

  ros::Subscriber sub = nh.subscribe("raw_obstacles", 10, dirCallback);
  ros::Subscriber eoc_sub = nh.subscribe("end_of_choreo", 1000, pauseCallback);
  ros::Subscriber queue_sub = nh.subscribe("queue_to_lidar", 1000, queueCallback);
  ros::Subscriber camera_sub = nh.subscribe("camera_to_lidar", 1000, cameraCallback);

  ros::NodeHandle oi;
  George = oi.advertise<std_msgs::String>("larry", 1000);
  audio_pub = oi.advertise<std_msgs::String>("audio_channel", 1000);
  update_hub_pub = oi.advertise<std_msgs::String>("from_chair", 1000);

  ros::waitForShutdown();

  return 0;
}
