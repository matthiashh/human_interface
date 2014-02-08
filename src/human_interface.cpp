#include "human_interface.h"
#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
//#include <human_interface/speechRequest.h> //doesn't work -.-
#include <std_msgs/String.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_interface");
  human_interface_class mainInterface;

  mainInterface.run();
  return 0;
}

human_interface_class::human_interface_class()
{
  //initialize ros-stuff
  pubRobotSounds_ = n_.advertise<sound_play::SoundRequest>("robotsound",100);
  subSpeechRequests_ = n_.subscribe("/human_interface/speechRequest", 10, &human_interface_class::speechRequestCallback_, this);

  //initialize speech-stuff
  speakersInUse_ = false;

}

void human_interface_class::recognitionConfirmation_(human_interface::RecognitionConfirmation::Request &req, human_interface::RecognitionConfirmation::Response &res)
{
  ROS_INFO("Recognition confirmation starting");
}

void human_interface_class::yesNoQuestion(human_interface::YesNoQuestion::Request &req, human_interface::YesNoQuestion::Response &res)
{
  ROS_INFO("Yes-No-Question asked");

  say_(req.question);

}

void human_interface_class::speechRequestCallback_(const std_msgs::String received_request)
{
    ROS_INFO("Received new speech request with content: %s",received_request.data.c_str());
    say_(received_request.data);
}

int human_interface_class::run()
{
  ros::Rate r(10);
  while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
}

int human_interface_class::say_(std::string text_to_say)
{
  sound_play::SoundRequest request;
  request.sound = -3;
  request.command = 1;
  request.arg = text_to_say;
  request.arg2 = "";

  pubRobotSounds_.publish(request);
  ROS_DEBUG("Published text-to-speech %s",text_to_say.c_str());
  return 0;
}
