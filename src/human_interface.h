#ifndef HUMAN_INTERFACE_H
#define HUMAN_INTERFACE_H
#include <ros/ros.h>            // general ros functionalities
#include <string>
#include <std_msgs/String.h>
//#include <human_interface/SpeechRequest.h>  //doesn't work -.-


class human_interface
{
private:
  //ros-stuff
  ros::NodeHandle n_;
  ros::Publisher pubRobotSounds_;
  ros::Subscriber subSpeechRequests_;

  //speech
  bool speakersInUse_;
  int say(std::string text_to_say);
public:
  human_interface();
  void speechRequestCallback_(const std_msgs::String received_request);
  int run();
};

#endif // HUMAN_INTERFACE_H
