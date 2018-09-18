#include "task2_visitor_actions.h"

CTask2VisitorActions::CTask2VisitorActions(const std::string &name, const std::string &name_space) : CModule(name,name_space),

tts("tts_module", this->module_nh.getNamespace()),
speech("echo_module",this->module_nh.getNamespace()),
nav_module("nav_module", this->module_nh.getNamespace()),
gripper_module("gripper_module", this->module_nh.getNamespace()),
head("head_module",this->module_nh.getNamespace()),
image_diff("image_diff_module", this->module_nh.getNamespace()),
play_motion("play_motion_module", this->module_nh.getNamespace())
{
  this->start_operation();
  this->state =  T2_INIT_ACTION;
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
	ROS_INFO("[TASK2] ActionSaySentence - Returning TRUE");
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
     return this->ActionSaySentence(this->config_.goodbye_sentence);
}



bool CTask2VisitorActions::ActionMoveHead(double pan_angle, double tilt_angle){
    static bool is_command_sent = false;
    if (!is_command_sent){
        this->head.move_to(pan_angle,tilt_angle);
        is_command_sent = true;
        this->current_action_retries_ = 0;
    }
    else {
      if (this->head.is_finished()){
        if (this->head.get_status()==HEAD_MODULE_SUCCESS or this->current_action_retries_ >= this->config_.max_action_retries){
          is_command_sent  = false;
          this->current_action_retries_ = 0;
          return true;
        }
        else {
          ROS_INFO ("[TASK2] HEAD module finished unsuccessfully. Retrying. Total retries = %d of %d", this->current_action_retries_, this->config_.max_action_retries);
          is_command_sent  = false;
          this->current_action_retries_ ++;
          return false;
        }

      }
    }
    return false;

}

void CTask2VisitorActions::StartActions(Person p){
    this->is_action_finished_ = false;
    this->visitor_ = p;
    this->start_actions_ = true;
    this->SetInitialStatesAllPersons();
}



void CTask2VisitorActions::state_machine(void)
{
    switch (this->state) {
        case T2_INIT_ACTION:
            if (this->start_actions_){
                if (this->visitor_ != Undefined){
                    this->state = T2_EXECUTING;
                }
            }
            break;
        case T2_EXECUTING:
            if (this->ExecuteBehaviorForVisitor(this->visitor_)){
 		this->is_action_finished_ = true;
                this->state = T2_END_ACTION;
            }
            break;
        case T2_END_ACTION:
            this->state = T2_INIT_ACTION;
            break;

    }

}

void CTask2VisitorActions::reconfigure_callback(task2_visitor_actions::Task2VisitorActionsConfig &config, uint32_t level)
{
  ROS_INFO("CTask2VisitorActions: reconfigure callback");
  this->lock();

  if (config.start_plumber){
      this->StartActions(Plumber);
      config.start_plumber = false;
  }
  if (config.start_kimble){
      this->StartActions(Kimble);
      config.start_kimble = false;
  }
  if (config.start_postman){
      this->StartActions(Postman);
      config.start_postman = false;
  }
  if (config.start_deliman){
      this->StartActions(Deliman);
      config.start_deliman = false;
  }
  this->config_=config;
  /* set the module rate */
//  this->set_rate(config.rate_hz);
  this->unlock();
}

void CTask2VisitorActions::stop(void){
    this->cancel_pending_ = true;
}

bool CTask2VisitorActions::is_finished(void){
    return this->is_action_finished_;
}

task2_action_states CTask2VisitorActions::get_state(void){
    return this->state;
}

void CTask2VisitorActions::SetInitialStatesAllPersons(){

            this->kimble_state = kimble_request_follow;
            this->deliman_state = deliman_request_follow_kitchen;
            this->postman_state = postman_ask_deliver;
            this->plumber_state = plumber_ask_destination;
}
bool CTask2VisitorActions::ExecuteBehaviorForVisitor(const Person & person){
    switch (person) {
        case Kimble:
            return KimbleStateMachine();
            break;
        case Deliman:
            return DelimanStateMachine();
        case Postman:
            return PostmanStateMachine();
        case Plumber:
            return PlumberStateMachine();
        default:
            return false;
            break;
    }
}



 bool CTask2VisitorActions::KimbleStateMachine(void){
     bool action_finished = false;

     switch (this->kimble_state) {
         case kimble_request_follow:
            ROS_INFO("[TASK2Actions] Requesting to follow");
            if (this->ActionSaySentence("Please follow me to the bedroom")){
                this->kimble_state = kimble_nav_bedroom;
            }
            break;
        case kimble_nav_bedroom:
            ROS_INFO("[TASK2Actions] Navigation to bedroom");
            if (this->ActionNavigate(this->config_.bedroom_poi)){
                this->kimble_state = kimble_say_wait_outside;
            }
            break;
        case kimble_say_wait_outside:
            ROS_INFO("[TASK2Actions] Saying TIAGo will wait outside");
            if (this->ActionSaySentence("I will wait for you outside")){
                this->kimble_state = kimble_go_outside;
            }
            break;
        case kimble_go_outside:
           ROS_INFO("[TASK2Actions] Navigation outside of bedroom");
            if (this->ActionNavigate(this->config_.bedroom_outside_poi)){
                this->kimble_state = kimble_move_head;
            }
            break;
        case kimble_move_head:

           ROS_INFO("[TASK2Actions] Moving head");
            if (this->ActionMoveHead(this->config_.head_pos_kimble_pan, this->config_.head_pos_kimble_tilt)){
                this->kimble_state = kimble_wait_leave;
                this->image_diff.set_reference_image();
                this->image_diff.clear_change();
            }
            break;
        case kimble_wait_leave:
            ROS_INFO("[TASK2Actions] Waiting for image change");
            if (this->image_diff.has_changed()){
                this->kimble_state = kimble_nav_door;
            }
            break;
        case kimble_nav_door:
            ROS_INFO("[TASK2Actions] Navigation to door");
            if (this->ActionNavigate(this->config_.door_poi)){
                this->kimble_state = kimble_say_goodbye;
            }
            break;
        case kimble_say_goodbye:
            ROS_INFO("[TASK2Actions] Saying goodbye");
            if (this->GenericSayGoodbye()){
                this->kimble_state = kimble_finish;
            }
            break;
        case kimble_finish:
            ROS_INFO("[TASK2Actions] END");
            action_finished = true;
            break;
     }

     return action_finished;
 }
 bool CTask2VisitorActions::DelimanStateMachine(void){
     bool action_finished = false;
     switch (this->deliman_state) {
         case deliman_request_follow_kitchen:
	    ROS_INFO("[TASK2Actions] Request follow");
            if (this->ActionSaySentence("Please follow me to the kitchen")){
                this->deliman_state = deliman_guide_kitchen;
            }
            break;
         case deliman_guide_kitchen:
	    ROS_INFO("[TASK2Actions] Navigate kitchen");
            if (this->ActionNavigate(this->config_.kitchen_poi)){
                this->deliman_state = deliman_request_deliver;
            }
            break;
         case deliman_request_deliver:
	    ROS_INFO("[TASK2Actions] Request deliver breakfast");
            if (this->ActionSaySentence("Please deliver the breakfast on the kitchen table")){
                this->deliman_state = deliman_request_follow_door;
            }
            break;
         case deliman_request_follow_door:
	    ROS_INFO("[TASK2Actions] Request follow to door");
             if (this->ActionSaySentence("Thanks. Please follow me to the door")){
                 this->deliman_state = deliman_guide_door;
             }
            break;
         case deliman_guide_door:
	    ROS_INFO("[TASK2Actions] Navigate to door");
             if (this->ActionNavigate(this->config_.door_poi)){
                 this->deliman_state = deliman_say_goodbye;
             }
            break;
         case deliman_say_goodbye:
	    ROS_INFO("[TASK2Actions] Say goodbye");
            if (this->GenericSayGoodbye()){
		this->deliman_state = deliman_finish;
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

        case postman_ask_deliver:
            ROS_INFO("[TASK2Actions]Requesting to deliver mail into hand");
            if (this->ActionSaySentence("Please put the mail in my hand")){
                this->postman_state = postman_wait_timer;
                this->timeout.start(ros::Duration(this->config_.wait_close_gripper));
            }
            break;
        case postman_wait_timer:
            ROS_INFO("[TASK2Actions]Waiting before closing gripper");
            if (this->timeout.timed_out()){
                this->postman_state = postman_close_gripper;
                this->gripper_module.close_grasp();
            }
            break;
        case postman_close_gripper:
            ROS_INFO("[TASK2Actions]Closing gripper");
            if (this->gripper_module.is_finished()){
                this->postman_state = postman_say_goodbye;
            }
            break;
        case postman_say_goodbye:
            ROS_INFO("[TASK2Actions] Saying goodbye");
            if (this->GenericSayGoodbye()){
                this->postman_state = postman_reach_bedroom;
            }
            break;
        case postman_reach_bedroom:
            ROS_INFO("[TASK2Actions] Navigation to bedroom");
            if (this->ActionNavigate(this->config_.bedroom_poi)){
                this->postman_state = postman_offer_gripper;
                this->play_motion.execute_motion(OFFER_GRIPPER_MOTION);
            }
            break;
        case postman_offer_gripper:
            if (this->play_motion.is_finished()){
                this->postman_state = postman_request_get_package;
            }
            break;
        case postman_request_get_package:
            ROS_INFO("[TASK2Actions] Request to take package");
            if (this->ActionSaySentence("Hello Granny Annie, please take the mail from my hand")){
                this->postman_state = postman_open_gripper;
                this->gripper_module.open();
            }
            break;
        case postman_open_gripper:
            if (this->gripper_module.is_finished()){
                this->postman_state = postman_finish;
            }
            break;
        case postman_finish:
            ROS_INFO("[TASK2Actions]END");
            action_finished = true;
            break;
     }

     return action_finished;
 }
 bool CTask2VisitorActions::PlumberStateMachine(void){
     bool action_finished = false;
     switch (this->plumber_state) {
        case plumber_ask_destination:
            ROS_INFO("[TASK2Actions] Asking for destination");
            if (this->ActionSaySentence(this->config_.sentence_which_room)){
                this->plumber_state = plumber_listen_destination;
                this->speech.listen();
            }
            break;
        case plumber_listen_destination:
            ROS_INFO("[TASK2Actions] Listening for destination");
            if (this->speech.is_finished()){
                if (this->speech.get_status()==ECHO_MODULE_SUCCESS){
                    this->speech_command_ = this->speech.get_result();
                    if (this->speech_command_.cmd.cmd_id == this->config_.speech_destination){
                        if (SetPOIDependingOnCommand(this->speech_command_.cmd.text_seq[0])){
                            this->plumber_state = plumber_request_follow;
                        }
                    }
                }
            }
            break;
        case plumber_request_follow:
            ROS_INFO("[TASK2Actions] Request to follow to the room");
            if (this->ActionSaySentence("Please follow me to the "+this->plumber_destination_name_)){
                this->plumber_state = plumber_nav_poi;
            }
            break;
        case plumber_nav_poi:
            ROS_INFO("[TASK2Actions] Navigation to destination");
            if (this->ActionNavigate(this->plumber_destination_poi_)){
                this->plumber_state = plumber_wait_leave;
            }
            break;
        case plumber_move_head:
            ROS_INFO("[TASK2Actions] Moving head");
            if (this->ActionMoveHead(this->config_.head_pos_plumber_pan,this->config_.head_pos_plumber_tilt)){
                this->plumber_state = plumber_wait_leave;
                this->image_diff.set_reference_image();
                this->image_diff.clear_change();
            }
            break;
        case plumber_wait_leave:
            ROS_INFO("[TASK2Actions] Waiting for plumber to leave");
            if (this->image_diff.has_changed()){
                this->plumber_state = plumber_nav_door;
            }
            break;
        case plumber_nav_door:
            ROS_INFO("[TASK2Actions] Navigation to door");
            if (this->ActionNavigate(this->config_.door_poi)){
                this->plumber_state = plumber_say_goodbye;
            }
            break;
        case plumber_say_goodbye:
            ROS_INFO("[TASK2Actions] Saying goodbye");
            if (this->GenericSayGoodbye()){
                this->plumber_state = plumber_finish;
            }
            break;
        case plumber_finish:
            ROS_INFO("[TASK2Actions] END");
            action_finished = true;
            break;
     }
     return action_finished;
 }

bool CTask2VisitorActions::SetPOIDependingOnCommand(const std::string & command_str){
    bool recognised_command = true;
    this->plumber_destination_name_ = command_str;
    if (command_str == this->config_.bathroom_name){
        this->plumber_destination_poi_ = config_.bathroom_poi;
    }
    else if (command_str == this->config_.kitchen_name){
        this->plumber_destination_poi_ = config_.kitchen_poi;
    }
    else {
        recognised_command = false;
    }
    return recognised_command;
}
