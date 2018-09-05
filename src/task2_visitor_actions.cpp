#include "task2_visitor_actions.h"

CTask2VisitorActions::CTask2VisitorActions(const std::string &name, const std::string &name_space) : CModule(name,name_space),

tts("tts_module",ros::this_node::getName()),
speech("echo_module",ros::this_node::getName()),
nav_module("nav_module", ros::this_node::getName()),
gripper_module("gripper_module", ros::this_node::getName())
{
  this->state =  T2_INIT;
  this->current_action_retries_ = 0;



}


CTask2VisitorActions::~CTask2VisitorActions(void)
{
  // [free dynamic memory]
}

bool CTask2VisitorActions::ActionNavigate(std::string & POI){
  static bool is_poi_sent = false;
  if (!is_poi_sent){
    nav_module.costmaps_clear();
    nav_module.go_to_poi(POI);
    is_poi_sent = true;
  }
  if (nav_module.is_finished()){
    if (nav_module.get_status()==NAV_MODULE_SUCCESS or this->current_action_retries_ >= this->config_.max_action_retries){
      is_poi_sent  = false;
      this->current_action_retries_ = 0;
      return true;

    }
    else {
      ROS_INFO ("[TASK2] Nav module finished unsuccessfully. Retrying");
      is_poi_sent  = false;
      this->current_action_retries_ ++;
      return false;
    }
  }
  return false;
}


bool CTask2VisitorActions::ActionSaySentence(const std::string & sentence){
  static bool is_sentence_sent = false;
  if (!is_sentence_sent){
    tts.say(sentence);
    is_sentence_sent = true;
  }
  else {
    if (tts.is_finished()){
      if (tts.get_status()==TTS_MODULE_SUCCESS or this->current_action_retries_ >= this->config_.max_action_retries){
        is_sentence_sent  = false;
        this->current_action_retries_ = 0;
        return true;

      }
      else {
        ROS_INFO ("[TASK2] TTS module finished unsuccessfully. Retrying. Total retries = %d of %d", this->current_action_retries_, this->config_.max_action_retries);
        is_sentence_sent  = false;
        this->current_action_retries_ ++;
        return false;
      }

    }
  }
  return false;
}

bool CTask2VisitorActions::GenericSayGoodbye (){
     return this->ActionSaySentence("Thanks for coming. Goodbye!");
}

void CTask2VisitorActions::StartActions(Person p){
    this->visitor_ = p;
    this->start_actions_ = true;
}



void CTask2VisitorActions::state_machine(void)
{
    switch (this->state) {
        case T2_INIT:
            if (this->start_actions_){
                if (this->visitor_ != Undefined){
                    this->state = T2_EXECUTING;
                }
            }
            break;
        case T2_EXECUTING:
            if (this->ExecuteBehaviorForVisitor(this->visitor_)){
                this->state = T2_END;
            }
            break;
        case T2_END:
            this->state = T2_INIT;
            break;

    }

}

void CTask2VisitorActions::reconfigure_callback(task2_visitor_actions::Task2VisitorActionsConfig &config, uint32_t level)
{
  ROS_INFO("CTask2VisitorActions: reconfigure callback");
  this->lock();
  this->config_=config;
  /* set the module rate */
//  this->set_rate(config.rate_hz);
  this->unlock();
}

void CTask2VisitorActions::stop(void){
    this->cancel_pending_ = true;
}

bool CTask2VisitorActions::is_finished(void){
    return false;
}

task2_action_states CTask2VisitorActions::get_state(void){
    return this->state;
}

bool CTask2VisitorActions::ExecuteBehaviorForVisitor(const Person & person){
    switch (person) {
        case Kimble:
            this->kimble_state = kimble_request_follow;
            return KimbleStateMachine();
            break;
        case Deliman:
            this->deliman_state = deliman_request_follow_kitchen;
            return DelimanStateMachine();
        case Postman:
            this->postman_state = postman_extend_arm;
            return PostmanStateMachine();
        case Plumber:
            this->plumber_state = plumber_ask_destination;
            return PlumberStateMachine();
        default:
            return false;
            break;
    }
}



 bool CTask2VisitorActions::KimbleStateMachine(void){

     return false;
 }
 bool CTask2VisitorActions::DelimanStateMachine(void){
     bool action_finished = false;
     switch (this->deliman_state) {
         case deliman_request_follow_kitchen:
            if (this->ActionSaySentence("Please follow me to the kitchen")){
                this->deliman_state = deliman_guide_kitchen;
            }
            break;
         case deliman_guide_kitchen:
            if (this->ActionNavigate(this->config_.kitchen_poi)){
                this->deliman_state = deliman_request_deliver;
            }
            break;
         case deliman_request_deliver:
            if (this->ActionSaySentence("Please deliver the breakfast on the kitchen table")){
                this->deliman_state = deliman_request_follow_door;
            }
            break;
         case deliman_request_follow_door:
             if (this->ActionSaySentence("Thanks. Please follow me to the door")){
                 this->deliman_state = deliman_request_follow_door;
             }
            break;
         case deliman_guide_door:
             if (this->ActionNavigate(this->config_.door_poi)){
                 this->deliman_state = deliman_say_goodbye;
             }
            break;
         case deliman_say_goodbye:
            if (this->GenericSayGoodbye()){

            }
            break;
         case deliman_finish:
            action_finished = true;
            break;
     }
     return action_finished;
 }
 bool CTask2VisitorActions::PostmanStateMachine(void){
     bool action_finished = false;
     switch (this->postman_state) {
        case postman_extend_arm:
            this->postman_state = postman_ask_deliver;
            break;
        case postman_ask_deliver:
            if (this->ActionSaySentence("Please put the mail in my hand")){
                this->postman_state = postman_close_gripper;
                this->gripper_module.close_grasp();
            }
            break;
        case postman_close_gripper:
            if (this->gripper_module.is_finished()){
                this->postman_state = postman_say_goodbye;
            }
            break;
        case postman_say_goodbye:
            if (this->GenericSayGoodbye()){
                this->postman_state = postman_reach_bedroom;
            }
            break;
        case postman_reach_bedroom:
            if (this->ActionNavigate(this->config_.bedroom_poi)){
                this->postman_state = postman_request_get_package;
            }
            break;
        case postman_request_get_package:
            if (this->ActionSaySentence("Hello Granny Annie, please take the mail from my hand")){
                this->postman_state = postman_finish;
            }
            break;
        case postman_finish:
            action_finished = true;
            break;
     }

     return action_finished;
 }
 bool CTask2VisitorActions::PlumberStateMachine(void){
     return false;
 }
