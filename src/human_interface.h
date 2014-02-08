#ifndef HUMAN_INTERFACE_H
#define HUMAN_INTERFACE_H
#include <ros/ros.h>            // general ros functionalities
#include <string>
#include <std_msgs/String.h>
#include <human_interface/SpeechRequest.h>
#include <human_interface/RecognitionConfirmation.h>
#include <human_interface/YesNoQuestion.h>




class human_interface_class
{
private:
  //ros-stuff
  ros::NodeHandle n_;
  ros::Publisher pubRobotSounds_;
  ros::Subscriber subSpeechRequests_;

  //speech
  bool speakersInUse_;
  int say_(std::string text_to_say);
  void recognitionConfirmation_(human_interface::RecognitionConfirmation::Request &req, human_interface::RecognitionConfirmation::Response &res);
  void yesNoQuestion(human_interface::YesNoQuestion::Request &req, human_interface::YesNoQuestion::Response &res);
public:
  human_interface_class();
  void speechRequestCallback_(const std_msgs::String received_request);
  int run();
};

#endif // HUMAN_INTERFACE_H
