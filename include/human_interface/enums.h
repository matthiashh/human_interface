 
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