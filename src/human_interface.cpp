#include "human_interface.h"
#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
//#include <human_interface/speechRequest.h> //doesn't work -.-
#include <std_msgs/String.h>
#include <person_detector/SpeechConfirmation.h>



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
  yesNoServer_ = n_.advertiseService("human_interface/yes_no_question",&human_interface_class::yesNoQuestionService,this);
  confirmationServer_ = n_.advertiseService("/human_interface/speech_confirmation", &human_interface_class::recognitionConfirmation_, this);
  subSpeechRecog_ = n_.subscribe("/recognizer/output",10,&human_interface_class::speechInputCallback_,this);
  pubConfirmations_ = n_.advertise<person_detector::SpeechConfirmation>("/human_interface/confirmations",10);
  //initialize speech-stuff
  speakersInUse_ = false;
  speech_confirmation_id = 0;
}

bool human_interface_class::recognitionConfirmation_(human_interface::RecognitionConfirmation::Request &req, human_interface::RecognitionConfirmation::Response &res)
{
  ROS_INFO("Recognition confirmation starting");
  if (req.name_array.size() == 0 || !getSpeakers(ros::Duration(15)))
    {
      res.label = "";
      res.sucessfull = false;
      return true;
    }
  person_detector::SpeechConfirmation query_to_detector;
  query_to_detector.header.frame_id = "/map";
  query_to_detector.header.stamp = ros::Time::now();
  query_to_detector.header.seq = speech_confirmation_id;
  speech_confirmation_id++;
  query_to_detector.id = req.recognition_id;
  query_to_detector.label = "";
  query_to_detector.running = true;
  pubConfirmations_.publish(query_to_detector);
  say_("Hey! My name is Max. I am searching for person.");
  std::string question;
  bool answer;
  int status;
  //go through the names
  for (unsigned int it = 0; it < req.name_array.size(); it++)
  {
    question = "Are you " + req.name_array[it] + "?";
    yesNoQuestion(question,answer,status);
    //process the result
    query_to_detector.header.stamp = ros::Time::now();
    switch (status)
    {
      case 0: //answered :-)
      {
        if (answer == true)
        {
          ROS_INFO("The question was answered, we can finish this confirmation.");
          say_("Yeah! I'm so happy, that I found you");
          res.sucessfull = true;
          query_to_detector.header.seq = speech_confirmation_id;
          speech_confirmation_id++;
          query_to_detector.running = false;
          query_to_detector.suceeded = false;
          query_to_detector.label = req.name_array[it];
          pubConfirmations_.publish(query_to_detector);
          return true;
        }
        else
        {
            ROS_INFO("This wasn't the right name, we go to the next");
            break;
        }
      }
      case 3: //speakers blocked
      {
        ROS_INFO("The speakers were blocked - we're aborting this confirmation");
        res.sucessfull = false;
        query_to_detector.header.seq = speech_confirmation_id;
        speech_confirmation_id++;
        query_to_detector.running = false;
        query_to_detector.suceeded = false;
        pubConfirmations_.publish(query_to_detector);
        speakersInUse_ = false;
        return true;
      }
      case 2: //wrong answer
      {
        ROS_INFO("The question wasn't properly answered - we're aborting this confirmation");
        res.sucessfull = false;
        query_to_detector.header.seq = speech_confirmation_id;
        speech_confirmation_id++;
        query_to_detector.running = false;
        query_to_detector.suceeded = false;
        pubConfirmations_.publish(query_to_detector);
        speakersInUse_ = false;
        return true;
      }
      case 1: // no answer at all
      {
        ROS_INFO("The question wasn't properly answered - we're aborting this confirmation");
        res.sucessfull = false;
        query_to_detector.header.seq = speech_confirmation_id;
        speech_confirmation_id++;
        query_to_detector.running = false;
        query_to_detector.suceeded = false;
        pubConfirmations_.publish(query_to_detector);
        speakersInUse_ = false;
        return true;
      }
    }
  }

}

bool human_interface_class::yesNoQuestionService(human_interface::YesNoQuestion::Request &req, human_interface::YesNoQuestion::Response &res)
{
  ROS_INFO("Yes-No-Question asked");
  getSpeakers(ros::Duration(15));
  std::string question = req.question;
  bool answer;
  int status;
  yesNoQuestion(question,answer,status);
  res.answer = answer;
  res.status = status;
  speakersInUse_ = false;
  return true;
}

void human_interface_class::yesNoQuestion(std::string question, bool &answer, int &status)
{
  say_(question);
  ros::Time done = ros::Time::now();
  //spin to get results
  ros::spinOnce();
  //wait until we have a result
  ros::Duration wait_for_result = ros::Duration(2);
  while (speech_q.empty())
    {
      wait_for_result.sleep();
      ros::spinOnce();
      if ((ros::Time::now() - done) > ros::Duration(15))
        {
          ROS_INFO("Aborting, because we didn't receive an answer.");
          status = 1;
          return;
        }
    }
  //throw away results which appeared too early
  ros::Duration wait = ros::Duration(1);
  bool cont = true;
  human_interface::speechRec act;
  while (cont)
  {
    if (!speech_q.empty())
    {
      act = speech_q.front();
      ROS_INFO("New sentence is %c",act.sentence.c_str());
      speech_q.pop();
      if (act.time < done)
      {
        //it appears to be earlier - throw it away
        continue;
      }
      else
      {
        //check for res and no
          if (act.sentence == "yes")
            {
              ROS_INFO("Question was answered with yes");
              answer = true;
              status = 0;
              return;
            }
          else if (act.sentence == "no")
            {
              ROS_INFO("Question was answered with no");
              answer = false;
              status = 0;
              return;
            }
          else
            {
              ROS_INFO("Throwing away the answer %c",act.sentence.c_str());
            }
      }
    }
    //sleep a second
    if ((ros::Time::now() - done) > ros::Duration(15))
    {
      //abort, because we waited too long
      ROS_INFO("Aborting because we didn't receive an right answer");
      status = 2;
      return;
    }
    else
    {
      wait.sleep();
      ros::spinOnce();
    }

  }
}

void human_interface_class::speechInputCallback_(const std_msgs::String speech)
{
  human_interface::speechRec new_one;
  new_one.time = ros::Time::now();
  new_one.sentence = speech.data;
  if (speech_q.size() > 10)
    {
      speech_q.pop();
    }
   speech_q.push(new_one);
}

void human_interface_class::speechRequestCallback_(const std_msgs::String received_request)
{
    ROS_INFO("Received new speech request with content: %s",received_request.data.c_str());
    if (!getSpeakers(ros::Duration(15))) return;
    say_(received_request.data);
    speakersInUse_ = false;
}

bool human_interface_class::getSpeakers(ros::Duration max)
{
  ros::Time start = ros::Time::now();
  ros::Duration sleep_time = ros::Duration(2,0);
  while (speakersInUse_)
  {
    sleep_time.sleep();
    if ((ros::Time::now()-start) > max)
    {
      ROS_INFO("Exiting speaker request, because the speaker seems to be blocked too long");
      return false;
    }
  }
  speakersInUse_ = true;
  return true;
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
  ros::Duration dur = ros::Duration(1.5);
  //experiments show 0.07 sec / letter
  dur += ros::Duration(text_to_say.size()*0.07);
  pubRobotSounds_.publish(request);
  ROS_DEBUG("Published text-to-speech %s",text_to_say.c_str());
  dur.sleep();
  return 0;
}
