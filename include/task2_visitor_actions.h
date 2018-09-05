// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// IMPORTANT NOTE: This code has been generated through a script from the
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _task2_visitor_actions_module_h
#define _task2_visitor_actions_module_h


//IRI ROS headers
#include <iri_ros_tools/module.h>
#include <iri_ros_tools/module_action.h>
#include <iri_ros_tools/module_service.h>
#include <iri_ros_tools/module_dyn_reconf.h>
#include <iri_ros_tools/watchdog.h>
#include <iri_ros_tools/timeout.h>


#include <tiago_modules/tts_module.h>
#include <tiago_modules/nav_module.h>
#include <tiago_modules/gripper_module.h>


#include <nen_modules/echo_module.h>
#include <nen_common_msgs/EchoCmdAction.h>


#include <task2_visitor_actions/Task2VisitorActionsConfig.h>


// [publisher subscriber headers]

// [service client headers]

// [action server client headers]


typedef enum {
    T2_INIT,
    T2_EXECUTING,
    T2_END
} task2_action_states;



typedef enum {
  kimble_request_follow,
  kimble_reach_bedroom,
  kimble_go_outside,
  kimble_move_head,
  kimble_wait_leave
} task2_kimble_states;

typedef enum{
    postman_extend_arm,
    postman_ask_deliver,
    postman_close_gripper,
    postman_say_goodbye,
    postman_reach_bedroom,
    postman_request_get_package,
    postman_finish
} task2_postman_states;

typedef enum {
    deliman_request_follow_kitchen,
    deliman_guide_kitchen,
    deliman_request_deliver,
    deliman_request_follow_door,
    deliman_guide_door,
    deliman_say_goodbye,
    deliman_finish
} task2_deliman_states;

typedef enum {
    plumber_ask_destination,
    plumber_listen_destination,
    plumber_request_follow,
    plumber_nav_poi,
    plumber_wait_leave,
    plumber_guide_door,
    plumber_say_goodbye,
    plumber_finish
} task2_plumber_states;

typedef enum {Deliman, Postman, Kimble, Plumber, Undefined} Person;

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class CTask2VisitorActions : public CModule<task2_visitor_actions::Task2VisitorActionsConfig>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    task2_visitor_actions::Task2VisitorActionsConfig config_;

  //Modules
    //Navigation module
    CNavModule nav_module;
    //Text to speech module
    CTTSModule tts;


    //Amazon echo modules for the plumber
    CEchoModule speech;
    nen_common_msgs::EchoCmdResult speech_command_;

    //Gripper module for the postman
    CGripperModule gripper_module;

    //Auxiliary variables to start task or ring bell from the dynamic_reconfigure

    bool cancel_pending_;
    bool start_actions_;
    //Variables for the delays
    Person visitor_;

    task2_postman_states postman_state;
    task2_kimble_states kimble_state;
    task2_plumber_states plumber_state;
    task2_deliman_states deliman_state;

    int current_action_retries_;

    //State machines
    task2_action_states state;



    /*!
       \brief Function that handles the tts module so that speaking is easier
       \param String with the sentence to be said by the robot
       \pre
       \post
       \return true if it ends saying the sentence, false otherwise.
    */
    bool ActionSaySentence(const std::string & sentence);
    bool ActionNavigate(std::string & POI);
    bool GenericSayGoodbye();

  protected:
    void state_machine(void);

   //Specific state machines
    bool KimbleStateMachine(void);
    bool DelimanStateMachine(void);
    bool PostmanStateMachine(void);
    bool PlumberStateMachine(void);


    void reconfigure_callback(task2_visitor_actions::Task2VisitorActionsConfig &config, uint32_t level);

   public:
     /**
      * \brief Constructor
      *
      * This constructor initializes specific class attributes and all ROS
      * communications variables to enable message exchange.
      */
      CTask2VisitorActions(const std::string &name, const std::string &name_space=std::string(""));




      void StartActions(Person person);


      /**
       * \brief Stops the current execution of the module
       *
       * This function stops the execution of the state machine, stopping all sub-processes in execution.
       *
       */
      void stop(void);

      /**
      * \brief Indicates if there is any execution in process
      *
      * This function checks if the state machine is in execution, what it means it is searching
      * and bringing a object.
      *
      * \param return a bollean indicating wether the last goal has been ended or not.
      */
      bool is_finished(void);

      /**
        * \brief Indicates the current status of the module
        *
        * This function returns the current status of the execution, which is a defined value
        * from task3_subtasks_status_t. It is useful to check if the goal has been completed
        * successfully or not when the execution of the module has just finished.
        *
        * \return the current status of the module.
        *
        */
       task2_action_states get_state(void);

       /*!
          \brief Function that executes the behaviour for the current visitor
          \param
          \pre visitor_ is defined
          \post  new step of the action for the visitor_ is done
          \return True when the action is finished.
       */
       bool ExecuteBehaviorForVisitor(const Person & person);




      /**
      * \brief Destructor
      *
      * This destructor frees all necessary dynamic memory allocated within this
      * this class.
      */
      ~CTask2VisitorActions(void);

};

#endif