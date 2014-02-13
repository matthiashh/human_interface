#ifndef HUMAN_INTERFACE_H
#define HUMAN_INTERFACE_H
#include <ros/ros.h>            // general ros functionalities
#include <string>
#include <std_msgs/String.h>
#include <human_interface/SpeechRequest.h>
#include <human_interface/RecognitionConfirmation.h>
#include <human_interface/YesNoQuestion.h>
#include <queue>                                        //to store speech recognition results


namespace human_interface {

  struct speechRec {
    ros::Time time;
    std::string sentence;
  };
}

class human_interface_class
{
private:
  //ros-stuff
  ros::NodeHandle n_;
  ros::Publisher pubRobotSounds_;
  ros::Subscriber subSpeechRequests_;
  ros::ServiceServer yesNoServer_;
  ros::Subscriber subSpeechRecog_;
  ros::ServiceServer confirmationServer_;
  ros::ServiceClient yesNoClient_;
  ros::Publisher pubConfirmations_;

  //speech
  bool speakersInUse_;
  int say_(std::string text_to_say);
  bool recognitionConfirmation_(human_interface::RecognitionConfirmation::Request &req, human_interface::RecognitionConfirmation::Response &res);
  bool yesNoQuestionService(human_interface::YesNoQuestion::Request &req, human_interface::YesNoQuestion::Response &res);
  void yesNoQuestion(std::string question, bool &answer, int &status);
  void speechInputCallback_ (const std_msgs::String speech);
  void speechRequestCallback_(const std_msgs::String received_request);
  std::queue <human_interface::speechRec> speech_q;
  bool getSpeakers(ros::Duration max);
  int speech_confirmation_id;
public:
  human_interface_class();
  int run();
};

#endif // HUMAN_INTERFACE_H
