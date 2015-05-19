#ifndef HUMAN_INTERFACE_H
#define HUMAN_INTERFACE_H
#include <ros/ros.h>            // general ros functionalities
#include <string>
#include <std_msgs/String.h>
#include <human_interface/SpeechRequest.h>
#include <human_interface/RecognitionConfirmation.h>
#include <human_interface/YesNoQuestion.h>
#include <queue>                                        //to store speech recognition results

//be carefull to modify the ./include/human_interface/enums.h as well
namespace human_interface {
  //! Used to add a time to incoming sentences of the speech recognition
  struct speechRec {
    ros::Time time;
    std::string sentence;
  };
  //! Defines the states an answer according to the message file
  enum yes_no_result {
    ANSWERED = 0,
    UNANSWERED = 1,
    WRONG_ANSWER = 2,
    BLOCKED_SPEAKER = 3
  };
}

//! Stand alone node to allow text-to-speech, process yes-no-questions and do confirmations
class human_interface_class
{
private:
  //ros-stuff
  ros::NodeHandle n_;                                       //!< Mandatory nodehandle
  ros::Publisher pubRobotSounds_;                           //!< Publisher for the text-to-speech soundplay-node
  ros::Subscriber subSpeechRequests_;                       //!< Subscriber to receive text-to-speech requests
  ros::ServiceServer yesNoServer_;                          //!< Server for yes-no-questions
  ros::Subscriber subSpeechRecog_;                          //!< Subscriber to the recognized speech outputs of pocketsphinx
  ros::ServiceServer confirmationServer_;                   //!< Server for confirmations

  //speech
  bool speakers_in_use_;                                    //!< True if the speakers are used (probably not need on a single thread program
  std::queue <human_interface::speechRec> speech_q;         //!< A queue for all recognized sentences with timestamps

  //! Forms the string to the tts-message and waits until its said
  /*! \param text_to_say Text which should be sent */
  void say_(std::string text_to_say);

  //! Server function for confirmation requests
  /*! \param req Received request
      \param res Returned result to the requesting client
      \return Passed to the client of the request */
  bool recognitionConfirmation(human_interface::RecognitionConfirmation::Request &req, human_interface::RecognitionConfirmation::Response &res);

  //! Server function for the yes-no-questions
  /*! \param req Received request
      \param res Returned result to the asking client
      \return Passed to the client of the request */
  bool yesNoQuestionService(human_interface::YesNoQuestion::Request &req, human_interface::YesNoQuestion::Response &res);

  //! Implementation of the yes-no-question
  /*! \param question Question string
      \param answer   True if the answer is yes
      \param status   Following the message definition */
  void yesNoQuestion(std::string question, bool &answer, int &status);

  //! Callback for received speech recognitions
  /*! \param speech The recognized sentence  */
  void speechRecognitionCallback_ (const std_msgs::String speech);

  //! Callback for speech requests
  /*! \param req Received request object */
  void speechRequestCallback_(human_interface::SpeechRequest req);

  //! Function to get access to the speaker
  /*! \todo Probably useless in a single threaded system
      \param max Maximal duration to wait
      \return true if access is granted */
  bool getSpeakers(ros::Duration max);
public:
  //! Constructor initializing subscriber and publisher
  human_interface_class();

  //! Runs the class and never exits except in critical errors
  int run();
};

#endif // HUMAN_INTERFACE_H
