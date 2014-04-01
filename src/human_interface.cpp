#include "human_interface.h"
#include <ros/ros.h>
#include <sound_play/SoundRequest.h>
//#include <human_interface/speechRequest.h> //doesn't work -.-
#include <std_msgs/String.h>




int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_interface");
  human_interface_class mainInterface;
  ROS_INFO("Finished initialization, now running in the loop");
  mainInterface.run();
  return 0;
}

human_interface_class::human_interface_class()
{
  //initialize ros-stuff
  pubRobotSounds_ = n_.advertise<sound_play::SoundRequest>("robotsound",100);
  subSpeechRequests_ = n_.subscribe("/human_interface/speech_request", 10, &human_interface_class::speechRequestCallback_, this);
  yesNoServer_ = n_.advertiseService("human_interface/yes_no_question",&human_interface_class::yesNoQuestionService,this);
  confirmationServer_ = n_.advertiseService("/human_interface/speech_confirmation", &human_interface_class::recognitionConfirmation, this);
  subSpeechRecog_ = n_.subscribe("/recognizer/output",10,&human_interface_class::speechRecognitionCallback_,this);
  //initialize speech-stuff
  speakers_in_use_ = false;
}

bool human_interface_class::recognitionConfirmation(human_interface::RecognitionConfirmation::Request &req, human_interface::RecognitionConfirmation::Response &res)
{
  ROS_INFO("Recognition confirmation received, now waiting for speaker");
  if (req.name_array.size() == 0 || !getSpeakers(ros::Duration(15)))
    {
      ROS_WARN("No access to speaker. Aborting this confirmation");
      res.label = "";
      res.sucessfull = false;
      return true;
    }

  say_("Hey! My name is Max. I am searching for person.");
  res.answered = false;
  std::string question;
  bool answer;
  int status;
  //go through the names
  for (unsigned int it = 0; it < req.name_array.size(); it++)
  {
    question = "Are you " + req.name_array[it] + "?";
    yesNoQuestion(question,answer,status);
    //process the result
    switch (status)
    {
      case human_interface::ANSWERED: //answered :-)
      {
        if (answer == true)
        {
          ROS_INFO("The question was answered, we can finish this confirmation.");
          res.sucessfull = true;
          res.answered = true;
          res.label = req.name_array[it];
          speakers_in_use_ = false;
          return true;
        }
        else
        {
            ROS_INFO("This wasn't the right name, we go to the next");
            res.answered = true;
            break;
        }
      }
      case human_interface::UNANSWERED: // no answer at all
      {
        ROS_INFO("The question wasn't properly answered - going on with the next name");
//        res.sucessfull = false;
//        query_to_detector.header.seq = speech_confirmation_id;
//        speech_confirmation_id++;
//        query_to_detector.running = false;
//        query_to_detector.suceeded = false;
//        pubConfirmations_.publish(query_to_detector);
//        speakers_in_use_ = false;
//        return true;
        break;
      }
      case human_interface::WRONG_ANSWER: //wrong answer
      {
        ROS_INFO("The question wasn't properly answered - we go on with the next name");
//        res.sucessfull = false;
//        query_to_detector.header.seq = speech_confirmation_id;
//        speech_confirmation_id++;
//        query_to_detector.running = false;
//        query_to_detector.suceeded = false;
//        pubConfirmations_.publish(query_to_detector);
//        speakers_in_use_ = false;
//        return true;
        break;
      }
      case human_interface::BLOCKED_SPEAKER: //speakers blocked - shouldn't happen, as we blocked the speakers before
      {
        ROS_ERROR("The speakers were blocked - we're aborting this confirmation");
        res.sucessfull = false;
        speakers_in_use_ = false;
        return true;
      }
    }
  }
  ROS_INFO("Gone through all the names and didn't receive a positive answer.");
  res.sucessfull = false;
  speakers_in_use_ = false;
  return true;
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
  speakers_in_use_ = false;
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
      //ROS_INFO("New sentence is %s",act.sentence.c_str());
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
              ROS_INFO("Throwing away the answer %s",act.sentence.c_str());
            }
      }
    }
    //sleep a second
    if ((ros::Time::now() - done) > ros::Duration(20))
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

void human_interface_class::speechRecognitionCallback_(const std_msgs::String speech)
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

void human_interface_class::speechRequestCallback_(human_interface::SpeechRequest req)
{
    ROS_INFO("Received new speech request");
    if (!getSpeakers(ros::Duration(15)))
      {
        ROS_WARN("Skipping speech request, because we don't have access to the speakers. Data: %s",req.text_to_say.c_str());
        return;
      }
    say_(req.text_to_say);
    speakers_in_use_ = false;
}

bool human_interface_class::getSpeakers(ros::Duration max)
{
  ros::Time start = ros::Time::now();
  ros::Duration sleep_time = ros::Duration(2,0);
  while (speakers_in_use_)
  {
    sleep_time.sleep();
    if ((ros::Time::now()-start) > max)
    {
      ROS_INFO("Exiting speaker request, because the speaker seems to be blocked too long");
      return false;
    }
  }
  speakers_in_use_ = true;
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

void human_interface_class::say_(std::string text_to_say)
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
  ROS_INFO("Published text-to-speech: %s",text_to_say.c_str());
  dur.sleep();
}
