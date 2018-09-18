cmake_minimum_required(VERSION 2.8.3)
project(task2_visitor_actions)

## Find catkin macros and libraries
# ********************************************************************
#                 Add catkin additional components here
# ********************************************************************
find_package(catkin REQUIRED COMPONENTS iri_ros_tools  tiago_modules nen_modules
task2_recognition log_modules)

find_package(iriutils REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ********************************************************************
#           Add system and labrobotica dependencies here
# ********************************************************************
# find_package(<dependency> REQUIRED)

# ********************************************************************
#           Add topic, service and action definition here
# ********************************************************************
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# ********************************************************************
#                 Add the dynamic reconfigure file
# ********************************************************************
generate_dynamic_reconfigure_options(
    cfg/Task2VisitorActions.cfg
)

# ********************************************************************
#                 Add run time dependencies here
# ********************************************************************
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES task2_visitor_actions
# ********************************************************************
#            Add ROS and IRI ROS run time dependencies
# ********************************************************************
 CATKIN_DEPENDS
 iri_ros_tools
 log_modules
 nen_modules
 tiago_modules
 task2_recognition
# ********************************************************************
#      Add system and labrobotica run time dependencies here
# ********************************************************************
)

###########
## Build ##
###########

# ********************************************************************
#                   Add the include directories
# ********************************************************************
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${iriutils_INCLUDE_DIR})
# include_directories(${<dependency>_INCLUDE_DIR})

## Declare a cpp library
 add_library(${PROJECT_NAME} src/task2_visitor_actions.cpp)

## Declare a cpp executable

# ********************************************************************
#                   Add the libraries
# ********************************************************************
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY})


# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})

# ********************************************************************
#               Add message headers dependencies
# ********************************************************************
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
# ********************************************************************
#               Add dynamic reconfigure dependencies
# ********************************************************************
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
