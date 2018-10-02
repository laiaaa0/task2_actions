#include "task2_visitor_actions.h"

CTask2VisitorActions::CTask2VisitorActions(const std::string &name, const std::string &name_space) : CModule(name,name_space),

tts("tts_module", this->module_nh.getNamespace()),
speech("echo_module",this->module_nh.getNamespace()),
nav_module("nav_module", this->module_nh.getNamespace()),
gripper_module("gripper_module", this->module_nh.getNamespace()),
head("head_module",this->module_nh.getNamespace()),
image_diff("image_diff_module", this->module_nh.getNamespace()),
logging("log_module", this->module_nh.getNamespace()),
play_motion("play_motion_module", this->module_nh.getNamespace()),
move_platform("move_platform",this->module_nh.getNamespace()),
guiding("guiding", this->module_nh.getNamespace()),
following("following_module", this->module_nh.getNamespace())


{
  this->start_operation();
  this->state =  T2_INIT_ACTION;
  this->status = T2_ACTIONS_MODULE_SUCCESS;
  this->current_action_retries_ = 0;
  this->current_follow_retries_ = 0;

  this->spencer_tracked_people_rear_subscriber_ = this->module_nh.subscribe("current_id_rear", 1, &CTask2VisitorActions::spencer_tracked_people_rear_callback, this);
  pthread_mutex_init(&this->spencer_tracked_people_rear_mutex_,NULL);
  boost::function<bool (int)> cb_rear (boost::bind(&CTask2VisitorActions::headsearch_callback_rear, this, _1));
  this->guiding.set_callback(cb_rear);

  this->spencer_tracked_people_front_subscriber_ = this->module_nh.subscribe("current_id_front", 1, &CTask2VisitorActions::spencer_tracked_people_front_callback, this);
  pthread_mutex_init(&this->spencer_tracked_people_front_mutex_,NULL);
  boost::function<bool (int)> cb_front (boost::bind(&CTask2VisitorActions::headsearch_callback_front, this, _1));
  this->following.set_callback(cb_front);

  this->visitor_ = Undefined;

}


CTask2VisitorActions::~CTask2VisitorActions(void)
{
  // [free dynamic memory]
}

void CTask2VisitorActions::spencer_tracked_people_rear_callback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg){
    ROS_DEBUG("CTask2VisitorActions::spencer_tracked_people_callback: New Message Received");
    this->tracked_persons_rear_ = msg->tracks;
    if (this->tracked_persons_rear_.size()>0){
        for (size_t i = 0; i < this->tracked_persons_rear_.size(); i++) {
            if (this->tracked_persons_rear_[i].is_matched){
                this->rear_spencer_detections = true;
            }
        }
    }
}
void CTask2VisitorActions::spencer_tracked_people_front_callback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg){
    ROS_DEBUG("CTask2VisitorActions::spencer_tracked_people_callback: New Message Received");
    this->tracked_persons_front_ = msg->tracks;
    if (this->tracked_persons_front_.size()>0){
        for (size_t i = 0; i < this->tracked_persons_front_.size(); i++) {
            if (this->tracked_persons_front_[i].is_matched){
                this->front_spencer_detections = true;
            }
        }
    }
}

bool CTask2VisitorActions::headsearch_callback_rear(const int id){
    ROS_INFO("[CTask2VisitorActions] Callback head search. ID = %d", id);
    for (int i = 0; i < this->tracked_persons_rear_.size(); ++i) {
      if (this->tracked_persons_rear_[i].track_id == id and this->tracked_persons_rear_[i].is_matched)
        return true;
    }
    return false;
}

bool CTask2VisitorActions::headsearch_callback_front(const int id){
    ROS_INFO("[CTask2VisitorActions] Callback head search");
    for (int i = 0; i < this->tracked_persons_front_.size(); ++i) {
      if (this->tracked_persons_front_[i].track_id == id and this->tracked_persons_front_[i].is_matched)
        return true;
    }
    return false;
}

bool CTask2VisitorActions::ActionGuide(std::string & POI){
  static bool is_poi_sent = false;
  if (!is_poi_sent){
        int id = DecideMainPersonID(GUIDING_MODE);
        ROS_INFO("Start GUIDING person = %d", id);
        if (id == -1){
            ROS_ERROR("No person detected");
            if (this->current_action_retries_ >= this->config_.max_action_retries){
                this->current_action_retries_ = 0;
                return true;
            }
            else {
                this->current_action_retries_++;
                return false;
            }
        }
        else {
            guiding.start(id, POI);
            is_poi_sent = true;
        }

  }
  if (guiding.is_finished()){
    if (guiding.get_status()==GUIDING_MODULE_SUCCESS or this->current_action_retries_ >= this->config_.max_action_retries){
      is_poi_sent  = false;
      this->current_action_retries_ = 0;
      return true;

    }
    else {
      ROS_INFO ("[TASK2Actions] Guiding module finished unsuccessfully. Retrying %d of %d", this->current_action_retries_,this->config_.max_action_retries);
      is_poi_sent  = false;
      this->current_action_retries_ ++;
      return false;
    }
  }
  return false;
}

bool CTask2VisitorActions::ActionFollow(){
  static bool is_command_sent = false;
  if (!is_command_sent){
    int id = DecideMainPersonID(FOLLOWING_MODE);
    if (id == -1){
        ROS_ERROR("No person detected");
        if (this->current_action_retries_ >= this->config_.max_action_retries){
            this->current_action_retries_ = 0;
            return true;
        }
        else {
            this->current_action_retries_ ++;
            return false;
        }
    }
    else {
        following.start(id);
        is_command_sent = true;
        this->speech.listen();
    }
  }
  if (this->speech.is_finished()){
    if (this->speech.get_status()==ECHO_MODULE_SUCCESS or this->current_action_retries_ >= this->config_.max_action_retries){
      int speech_cmd_id = this->speech.get_result().cmd.cmd_id;
      if (speech_cmd_id == this->config_.speech_stop_follow){
          this->following.stop();
          is_command_sent  = false;
          this->current_action_retries_ = 0;
          return true;
      }
      else {
          is_command_sent  = false;
          this->current_action_retries_ ++;
          return false;
      }
    }
    else {
      ROS_INFO ("[TASK2] Following module finished unsuccessfully. Retrying");
      is_command_sent  = false;
      this->current_action_retries_ ++;
      return false;
    }
  }
  return false;
}


double CTask2VisitorActions::DistanceFromPerson(const geometry_msgs::Point & position){
    return sqrt(pow(position.x, 2.0) + pow(position.y, 2.0));
}

int CTask2VisitorActions::DecideMainPersonID(const MOVING_MODE move_type){
    std::vector<spencer_tracking_msgs::TrackedPerson> all_detections;
    if (move_type == GUIDING_MODE){
        all_detections = tracked_persons_rear_;
    }
    else if (move_type == FOLLOWING_MODE){
        all_detections = tracked_persons_front_;
    }

    ROS_INFO("Deciding main person. Num detections= %d", all_detections.size());

    if (all_detections.size() < 1 ){
        return -1;
    }

    if (all_detections.size() == 1 ){
        return all_detections[0].track_id;
    }
    else {
        double min_distance = this->DistanceFromPerson(all_detections[0].pose.pose.position);
        int min_id = 0;
        for (size_t i = 0; i < all_detections.size(); i++) {
            double current_distance = this->DistanceFromPerson(all_detections[i].pose.pose.position);
            if (current_distance < min_distance || (all_detections[i].is_matched && !all_detections[min_id].is_matched)){
                    min_distance = current_distance;
                    min_id = i;
            }
        }
        return min_id;
    }
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
      ROS_INFO ("[TASK2] Nav module finished unsuccessfully. Retrying %d of %d",
                this->current_action_retries_,
                this->config_.max_action_retries);
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
    this->logging.start_logging_audio();
    is_sentence_sent = true;
  }
  else {
    if (tts.is_finished()){

      if (tts.get_status()==TTS_MODULE_SUCCESS or this->current_action_retries_ >= this->config_.max_action_retries){
        is_sentence_sent  = false;
        this->current_action_retries_ = 0;
        this->logging.stop_logging_audio();
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

bool CTask2VisitorActions::ActionTurnAround(){

      static bool is_command_sent = false;
      if (!is_command_sent){
        this->move_platform.orientate_platform(this->config_.angle_180);
        is_command_sent = true;
      }
      else {
        if (this->move_platform.is_finished()){
          if (this->move_platform.get_status()==MOVE_PLATFORM_MODULE_SUCCESS or this->current_action_retries_ >= this->config_.max_action_retries){
            is_command_sent  = false;
            this->current_action_retries_ = 0;
            return true;
          }
          else {
            ROS_INFO ("[TASK2] move_platform module finished unsuccessfully. Retrying. Total retries = %d of %d", this->current_action_retries_, this->config_.max_action_retries);
            is_command_sent  = false;
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
    this->status = T2_ACTIONS_MODULE_RUNNING;
    this->SetInitialStatesAllPersons();
}



void CTask2VisitorActions::state_machine(void)
{
    if (this->cancel_pending_){
        this->cancel_pending_ = false;
        this->state = T2_END_ACTION;
        this->tts.stop();
        this->nav_module.stop();
        this->head.stop();
        this->move_platform.stop();
        this->gripper_module.stop();
        this->play_motion.stop();
        this->status = T2_ACTIONS_MODULE_STOPPED;
    }
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
            this->postman_state = postman_init;
            this->plumber_state = plumber_ask_destination;
}
bool CTask2VisitorActions::ExecuteBehaviorForVisitor(const Person & person){
    switch (person) {
        case Kimble:
            return KimbleStateMachine();
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
                this->kimble_state = kimble_turn_around_door;
            }
            break;
        case kimble_turn_around_door:
            ROS_INFO("[TASK2Actions] Turn around");
            if (this->ActionTurnAround()){
                    this->kimble_state = kimble_wait_detect;
                    this->rear_spencer_detections = false;
                    this->timeout.start(ros::Duration(this->config_.wait_for_detections));

            }
            break;
        case kimble_wait_detect:
            ROS_INFO("[TASK2Actions] Wait detection");
            if (this->rear_spencer_detections){
                this->kimble_state = kimble_nav_bedroom;
            }

            else {
                if (this->timeout.timed_out()){
                    this->tts.say("I could not find you. Goodbye");
                    this->kimble_state = kimble_finish;
                }
            }
            break;
        case kimble_nav_bedroom:
            ROS_INFO("[TASK2Actions] Navigation to bedroom");
            if (this->ActionGuide(this->config_.bedroom_poi)){
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
                this->kimble_state = kimble_move_head_down;
            }
            break;
        case kimble_move_head_down:

           ROS_INFO("[TASK2Actions] Moving head");
            if (this->ActionMoveHead(this->config_.head_pos_kimble_pan, this->config_.head_pos_kimble_tilt)){
                this->kimble_state = kimble_wait_leave;
                this->image_diff.set_reference_image();
                this->image_diff.clear_change();
                this->timeout.start(ros::Duration(this->config_.wait_leave_room));
            }
            break;
        case kimble_wait_leave:
            ROS_INFO("[TASK2Actions] Waiting for image change");
            if (this->image_diff.has_changed()){
                this->kimble_state = kimble_move_head_up;
            }
            else {
                if (this->timeout.timed_out()){
                    this->kimble_state = kimble_move_head_up;
                }
            }
            break;
        case kimble_move_head_up:
           ROS_INFO("[TASK2Actions] Moving head");
            if (this->ActionMoveHead(0.0, 0.0)){
                this->kimble_state = kimble_ask_move_front;
                this->current_follow_retries_ = 0;

            }
            break;
        case kimble_ask_move_front:
            ROS_INFO("[TASK2Actions] Asking move in front");
            if (this->ActionSaySentence("Please move in front of me")){
                this->kimble_state = kimble_wait_detect_front;
                this->front_spencer_detections = false;
                this->timeout.start(ros::Duration(this->config_.wait_for_detections));
            }
            break;
        case kimble_wait_detect_front:
            ROS_INFO("[TASK2Actions] Waiting detection");
            if (this->front_spencer_detections){
                this->kimble_state = kimble_nav_door;

            }
            else {
                if (this->timeout.timed_out()){
                    this->tts.say("I could not find you. Goodbye");
                    this->kimble_state = kimble_finish;
                }
            }
            break;
        case kimble_nav_door:
            ROS_INFO("[TASK2Actions] Navigation to door");
            if (this->ActionFollow()){
                this->kimble_state = kimble_check_follow_ok;
            }
            break;
        case kimble_check_follow_ok:
            ROS_INFO("[TASK2Actions] Checking follow ok");
            if (this->following.is_finished()){
                if (this->following.get_status() == FOLLOWING_MODULE_SUCCESS){
                    this->kimble_state = kimble_say_goodbye;
                }
                else {
                    if (this->following.get_status() == FOLLOWING_MODULE_FAILURE){
                        if (this->current_follow_retries_ > this->config_.max_follow_retries){
                            this->kimble_state = kimble_say_goodbye;
                        }
                        else {
                            this->kimble_state = kimble_ask_move_front;
                            this->current_follow_retries_ ++;

                        }
                    }
                }
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
            this->status = T2_ACTIONS_MODULE_SUCCESS;
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
                this->deliman_state = deliman_turn_around_door;
            }
            break;
         case deliman_turn_around_door:
            ROS_INFO("[Task2Actions] Turning around");
            if (this->ActionTurnAround()){
                this->deliman_state = deliman_wait_detect_1;
                this->rear_spencer_detections = false;
                this->timeout.start(ros::Duration(this->config_.wait_for_detections));

            }
            break;
         case deliman_wait_detect_1:
            ROS_INFO("[Task2Actions] Waiting detection");

            if (this->rear_spencer_detections){
                this->deliman_state = deliman_guide_kitchen;
            }

            else {
                if (this->timeout.timed_out()){
                    this->tts.say("I could not find you. Goodbye");
                    this->deliman_state = deliman_finish;
                }
            }
            break;
         case deliman_guide_kitchen:
	    ROS_INFO("[TASK2Actions] Navigate kitchen");
            if (this->ActionGuide(this->config_.kitchen_poi)){
                this->deliman_state = deliman_request_deliver;
            }
            break;
         case deliman_request_deliver:
	    ROS_INFO("[TASK2Actions] Request deliver breakfast");
            if (this->ActionSaySentence("Please deliver the breakfast on the kitchen table")){
                this->deliman_state = deliman_head_normal;
            }
            break;
        case deliman_head_normal:
            ROS_INFO("[TASK2Actions] Moving head");
            if (this->ActionMoveHead(0.0,0.0)){
                this->deliman_state = deliman_request_follow_door;
            }
            break;
         case deliman_request_follow_door:
	    ROS_INFO("[TASK2Actions] Request follow to door");
             if (this->ActionSaySentence("Thanks. Please stand behind me")){
                 this->deliman_state = deliman_wait_detect_2;
                 this->rear_spencer_detections = false;
                 this->timeout.start(ros::Duration(this->config_.wait_for_detections));

             }
            break;
        case deliman_wait_detect_2:
            ROS_INFO("[TASK2Actions] Waiting detections");
            if (this->rear_spencer_detections){
                this->deliman_state = deliman_guide_door;

            }

            else {
                if (this->timeout.timed_out()){
                    this->tts.say("I could not find you. Goodbye");
                    this->deliman_state = deliman_finish;
                }
            }
            break;
         case deliman_guide_door:
	        ROS_INFO("[TASK2Actions] Navigate to door");
            if (this->ActionGuide(this->config_.door_poi)){
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
            this->status = T2_ACTIONS_MODULE_SUCCESS;
            action_finished = true;
            break;
     }
     return action_finished;
 }
 bool CTask2VisitorActions::PostmanStateMachine(void){
     bool action_finished = false;
     switch (this->postman_state) {
         case postman_init:
            ROS_INFO("[TASK2Actions]Postman init");
            this->gripper_module.close();
            this->postman_state = postman_wait_close_gripper_1;
            break;
        case postman_wait_close_gripper_1:
            ROS_INFO("[TASK2Actions]Postman close gripper");
            if (this->gripper_module.is_finished()){
                if (this->gripper_module.get_status() == NEN_GRIPPER_MODULE_SUCCESS){
                    this->play_motion.execute_motion(OFFER_GRIPPER_MOTION);
                    this->postman_state = postman_wait_offer_gripper_1;
                }
            }
            break;
        case postman_wait_offer_gripper_1:
            ROS_INFO("[TASK2Actions] Postman wait offer gripper");
            if (this->play_motion.is_finished()){
                if (this->play_motion.get_status() == PLAY_MOTION_MODULE_SUCCESS){
                        this->gripper_module.open();
                        this->postman_state = postman_wait_open_gripper_1;
                }
            }
            break;
        case postman_wait_open_gripper_1:
            ROS_INFO("[TASK2Actions]Postman open gripper");
            if (this->gripper_module.is_finished()){
                if (this->gripper_module.get_status() == NEN_GRIPPER_MODULE_SUCCESS){
                        this->postman_state = postman_ask_deliver;
                }
            }
            break;
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
                this->postman_state = postman_close_gripper_2;
                this->gripper_module.close();
            }
            break;
        case postman_close_gripper_2:
            ROS_INFO("[TASK2Actions]Closing gripper");
            if (this->gripper_module.is_finished()){
                this->postman_state = postman_say_goodbye;
            }
            break;
        case postman_say_goodbye:
            ROS_INFO("[TASK2Actions] Saying goodbye");
            if (this->GenericSayGoodbye()){
                this->play_motion.execute_motion(HOME_MOTION);
                this->postman_state = postman_arm_home_1;
            }
            break;
        case postman_arm_home_1:
            ROS_INFO("[TASK2Actions] Moving arm to home");
            if (this->play_motion.is_finished()){
                if (this->play_motion.get_status() == PLAY_MOTION_MODULE_SUCCESS){
                        this->postman_state = postman_reach_bedroom;
                }
            }
            break;
        case postman_reach_bedroom:
            ROS_INFO("[TASK2Actions] Navigation to bedroom");
            if (this->ActionNavigate(this->config_.bedroom_poi)){
                this->postman_state = postman_wait_offer_gripper_2;
                this->play_motion.execute_motion(OFFER_GRIPPER_MOTION);
            }
            break;
        case postman_wait_offer_gripper_2:
            if (this->play_motion.is_finished()){
                this->postman_state = postman_request_get_package;
            }
            break;
        case postman_request_get_package:
            ROS_INFO("[TASK2Actions] Request to take package");
            if (this->ActionSaySentence("Hello Granny Annie, please take the mail from my hand")){
                this->postman_state = postman_open_gripper_2;
                this->gripper_module.open();
            }
            break;
        case postman_open_gripper_2:
            ROS_INFO("[TASK2Actions]Open gripper");
            if (this->gripper_module.is_finished()){
                if (this->gripper_module.get_status() == NEN_GRIPPER_MODULE_SUCCESS){
                    this->postman_state = postman_close_gripper_3;
                    this->gripper_module.close();
                }
            }
            break;

        case postman_close_gripper_3:
            ROS_INFO("[TASK2Actions]Close gripper");
            if (this->gripper_module.is_finished()){
                if (this->gripper_module.get_status() == NEN_GRIPPER_MODULE_SUCCESS){
                    this->postman_state = postman_arm_home_2;
                    this->play_motion.execute_motion(HOME_MOTION);
                }
            }
            break;
        case postman_arm_home_2:
            ROS_INFO("[TASK2Actions]Arm to home");
            if (this->play_motion.is_finished()){
                if (this->play_motion.get_status() == PLAY_MOTION_MODULE_SUCCESS){
                    this->postman_state = postman_finish;
                }
            }
            break;
        case postman_finish:
            ROS_INFO("[TASK2Actions]END");
            this->status = T2_ACTIONS_MODULE_SUCCESS;
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
                this->logging.start_logging_audio();
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
                            this->logging.stop_logging_audio();
                            this->plumber_state = plumber_turn_around_door;
                        }
                    }
                }
            }
            break;

        case plumber_turn_around_door:
            ROS_INFO("[TASK2Actions] Turn around");
            if (this->ActionTurnAround()){
                this->plumber_state = plumber_wait_detection;
                this->rear_spencer_detections = false;
                this->timeout.start(ros::Duration(this->config_.wait_for_detections));

            }
            break;
        case plumber_wait_detection:
            ROS_INFO("[TASK2Actions] Wait detection");
            if (this->rear_spencer_detections){
                this->plumber_state = plumber_nav_poi;
            }
            else {
                if (this->timeout.timed_out()){
                    this->tts.say("I could not find you. Goodbye");
                    this->plumber_state = plumber_finish;
                }
            }
            break;
        case plumber_nav_poi:
            ROS_INFO("[TASK2Actions] Navigation to destination");
            if (this->ActionGuide(this->plumber_destination_poi_)){
                this->plumber_state = plumber_move_head_down;
            }
            break;
        case plumber_move_head_down:
            ROS_INFO("[TASK2Actions] Moving head");
            if (this->ActionMoveHead(this->config_.head_pos_plumber_pan,this->config_.head_pos_plumber_tilt)){
                this->plumber_state = plumber_wait_leave;
                this->image_diff.set_reference_image();
                this->image_diff.clear_change();
                this->timeout.start(ros::Duration(this->config_.wait_leave_room));

            }
            break;
        case plumber_wait_leave:
            ROS_INFO("[TASK2Actions] Waiting for plumber to leave");
            if (this->image_diff.has_changed()){
                this->plumber_state = plumber_move_head_up;
            }

            else {
                if (this->timeout.timed_out()){
                    this->plumber_state = plumber_move_head_up;
                }
            }
            break;
        case plumber_move_head_up:
            ROS_INFO("[TASK2Actions] Moving head");
            if (this->ActionMoveHead(0.0,0.0)){
                this->plumber_state = plumber_ask_move_front;
                this->current_follow_retries_ = 0;
            }

            break;
        case plumber_ask_move_front:
            ROS_INFO("[TASK2Actions] Asking move in front");
            if (this->ActionSaySentence("Please move in front of me")){
                this->plumber_state = plumber_wait_detect_front;
                this->front_spencer_detections = false;
                this->timeout.start(ros::Duration(this->config_.wait_for_detections));            }
            break;
        case plumber_wait_detect_front:
            if (this->front_spencer_detections){
                this->plumber_state = plumber_nav_door;

            }
            else {
                if (this->timeout.timed_out()){
                    this->tts.say("I could not find you. Goodbye");
                    this->plumber_state = plumber_finish;
                }
            }
            break;
        case plumber_nav_door:
            ROS_INFO("[TASK2Actions] Navigation to door");
            if (this->ActionFollow()){
                this->plumber_state = plumber_check_follow_ok;
            }
            break;
        case plumber_check_follow_ok:
            ROS_INFO("[TASK2Actions] Checking follow ok");
            if (this->following.is_finished()){
                if (this->following.get_status() == FOLLOWING_MODULE_SUCCESS){
                    this->plumber_state = plumber_say_goodbye;
                }
                else {
                    if (this->following.get_status() == FOLLOWING_MODULE_FAILURE){
                        if (this->current_follow_retries_ > this->config_.max_follow_retries){
                            this->plumber_state = plumber_say_goodbye;
                        }
                        else {
                            this->plumber_state = plumber_ask_move_front;
                            this->current_follow_retries_ ++;
                        }
                    }
                }
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
            this->status = T2_ACTIONS_MODULE_SUCCESS;
            action_finished = true;
            break;
     }
     return action_finished;
 }

bool CTask2VisitorActions::SetPOIDependingOnCommand(const std::string & command_str){
    bool recognised_command = true;
    if (command_str == this->config_.bathroom_name){
        this->plumber_destination_poi_ = config_.bathroom_poi;
    }
    else if (command_str == this->config_.kitchen_name){
        this->plumber_destination_poi_ = config_.kitchen_door_poi;
    }
    else if (command_str == this->config_.bedroom_name){
        //TODO: since the plumber cant go to the bedroom, if the name is bedroom, go to the bathroom
        this->plumber_destination_poi_ = config_.bathroom_poi;
    }
    else {
        recognised_command = false;
    }
    return recognised_command;
}
