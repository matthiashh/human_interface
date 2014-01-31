#include "human_interface.h"
#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
//#include <human_interface/speechRequest.h> //doesn't work -.-
#include <std_msgs/String.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_interface");
  human_interface mainInterface;

  mainInterface.run();
  return 0;
}

human_interface::human_interface()
{
  //initialize ros-stuff
  pubRobotSounds_ = n_.advertise<sound_play::SoundRequest>("robotsound",100);
  subSpeechRequests_ = n_.subscribe("/human_interface/speechRequest", 10, &human_interface::speechRequestCallback_, this);

  //initialize speech-stuff
  speakersInUse_ = false;

}

void human_interface::speechRequestCallback_(const std_msgs::String received_request)
{
    ROS_INFO("Received new speech request with content: %s",received_request.data.c_str());
    say(received_request.data);
}

int human_interface::run()
{
  ros::Rate r(10);
  while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
}

int human_interface::say(std::string text_to_say)
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
