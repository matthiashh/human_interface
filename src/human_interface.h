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

  enum yes_no_result {
    ANSWERED = 0,
    UNANSWERED = 1,
    WRONG_ANSWER = 2,
    BLOCKED_SPEAKER = 3
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

  //speech
  bool speakers_in_use_;
  int say_(std::string text_to_say);
  bool recognitionConfirmation_(human_interface::RecognitionConfirmation::Request &req, human_interface::RecognitionConfirmation::Response &res);
  bool yesNoQuestionService(human_interface::YesNoQuestion::Request &req, human_interface::YesNoQuestion::Response &res);
  void yesNoQuestion(std::string question, bool &answer, int &status);
  void speechRecognitionCallback_ (const std_msgs::String speech);
  void speechRequestCallback_(human_interface::SpeechRequest req);
  std::queue <human_interface::speechRec> speech_q;
  bool getSpeakers(ros::Duration max);
public:
  human_interface_class();
  int run();
};

#endif // HUMAN_INTERFACE_H
