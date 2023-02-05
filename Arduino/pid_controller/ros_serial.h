//#ifndef ROS_Serial_h
//#define ROS_Serial_h
//
//#include <Arduino.h>
//
//#include <ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>
//#include <eyes/Generic.h>
//
//ros::NodeHandle nh;
//
//// init to zero and update with each encoder tick
//std_msgs::Int32 int32_msg_R;
//std_msgs::Int32 int32_msg_L;
//ros::Publisher pubR("encoder_value_R", &int32_msg_R);
//ros::Publisher pubL("encoder_value_L", &int32_msg_L);
//
//
//
//  void generic_callback(const eyes::Generic& generic_msg) {
//     if (generic_msg.left_forward) {
//       setLeftMotorDir(FWD);
//     }
//     else {
//       setLeftMotorDir(BWD);
//     }
//     if (generic_msg.right_forward) {
//       setRightMotorDir(FWD);
//     }
//     else {
//       setRightMotorDir(BWD);
//     }
//     analogWrite(LEFT_MOTOR, generic_msg.left_speed);
//     analogWrite(RIGHT_MOTOR, generic_msg.right_speed);
//  
//     return;
//  };
//
//  void initROSSerial(ros::Subscriber<eyes::Generic>& generic_sub) {
//    nh.initNode();
//    nh.advertise(pubR);
//    nh.advertise(pubL);
//    nh.subscribe(generic_sub);
//  };
//                      
//
//#endif
