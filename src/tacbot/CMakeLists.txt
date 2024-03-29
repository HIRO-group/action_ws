cmake_minimum_required(VERSION 3.4)
project(tacbot)


set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(nlohmann_json 3.7.3 REQUIRED)

# set(CMAKE_LIBRARY_PATH ${HOME}/tacbot_planning/open3d-devel-linux-x86_64-cxx11-abi-0.18.0)
# find_library(NAMES gtest)
# find_package(Open3D REQUIRED)

# find_package(Open3D REQUIRED PATHS "/home/marmot/tacbot_planning/open3d-devel-linux-x86_64-cxx11-abi-0.18.0")


# find_library(OPEN3D_LIBRARY
#     NAMES Open3D
#     HINTS "${HOME}/tacbot_planning/open3d-devel-linux-x86_64-cxx11-abi-0.18.0"
# )

find_package(catkin REQUIRED
  COMPONENTS
    # interactive_markers
    moveit_core
    # moveit_ros_planning
    moveit_planners_ompl
    # moveit_ros_planning_interface
    moveit_ros_perception
    # rviz_visual_tools
    # moveit_visual_tools
    # pluginlib
    # controller_interface
    # geometric_shapes
    # message_generation
    # realtime_tools
    roscpp
    # rospy
    # franka_hw
    # franka_msgs
    # hardware_interface
    # eigen_conversions
    # joint_limits_interface
    geometry_msgs
    shape_msgs
    # tf
    # tf_conversions
    # visualization_msgs
    # pcl_conversions
    rosbag
    sensor_msgs
    # tf2
    # tf2_ros
    pcl_ros
    trac_ik_lib
    nlopt
    my_moveit_context
    # tf2_eigen
    # tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ompl REQUIRED)
find_package(Franka REQUIRED)
# find_package(PCL 1.2 REQUIRED)
find_package(NLopt REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   JointTorqueComparison.msg
#   TrajExecutionMonitor.msg
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
#   # franka_msgs
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include
LIBRARIES
  ${nlopt_LIBRARY}
CATKIN_DEPENDS
  # moveit_core
  # moveit_visual_tools
  # moveit_ros_planning_interface
  # interactive_markers
  # trac_ik_lib
  my_moveit_context
DEPENDS
  EIGEN3
  OMPL
  Franka
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(PUBLIC
  include
)

include_directories(SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${nlopt_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(base_planning SHARED src/base_planner.cpp)
add_library(perception_planning SHARED src/perception_planner.cpp)
add_library(contact_planning SHARED src/contact_planner.cpp src/visualizer_data.cpp)
add_library(contact_perception SHARED src/contact_perception.cpp)
add_library(visualizer SHARED src/visualizer.cpp src/visualizer_data.cpp)
add_library(panda_interface SHARED src/panda_interface.cpp)
add_library(common STATIC src/common.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(contact_planning ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(publish_pc_bag src/publish_pc_bag.cpp)
add_executable(broadcast_tf src/broadcast_tf.cpp)
add_executable(plan_and_execute src/plan_and_execute.cpp src/utilities.cpp)
add_executable(joint_knot_plan src/joint_knot_plan.cpp src/utilities.cpp)
add_executable(generate_contact_plan src/generate_contact_plan.cpp src/utilities.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"

## Add cmake target dependencies of the executable
## same as for the library above

add_definitions(${PCL_DEFINITIONS})

## Specify libraries to link a library or executable target against
target_link_libraries(visualizer PUBLIC ${catkin_LIBRARIES})
target_link_libraries(contact_perception PUBLIC
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # Open3D::Open3D
  )

target_link_libraries(base_planning PUBLIC ${catkin_LIBRARIES} visualizer)
target_link_libraries(contact_planning PUBLIC ${catkin_LIBRARIES} base_planning contact_perception)
target_link_libraries(perception_planning PUBLIC ${catkin_LIBRARIES} base_planning contact_perception visualizer)


target_link_libraries(common PUBLIC  ${catkin_LIBRARIES} ${Franka_LIBRARIES})
target_link_libraries(panda_interface PUBLIC ${catkin_LIBRARIES} ${Franka_LIBRARIES} ${nlopt_LIBRARY} ruckig common)


target_link_libraries(publish_pc_bag ${catkin_LIBRARIES})
target_link_libraries(broadcast_tf ${catkin_LIBRARIES})

target_link_libraries(plan_and_execute
  ${catkin_LIBRARIES}
  perception_planning
  visualizer
  panda_interface
)

target_link_libraries(joint_knot_plan
  ${catkin_LIBRARIES}
  perception_planning
  visualizer
  panda_interface
  nlohmann_json::nlohmann_json
)

target_link_libraries(generate_contact_plan
  ${catkin_LIBRARIES}
  contact_planning
  visualizer
  panda_interface
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

install(
TARGETS
  base_planning
  contact_planning
  perception_planning
  contact_perception
  visualizer
  panda_interface
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
TARGETS
  publish_pc_bag
  broadcast_tf
  plan_and_execute
  joint_knot_plan
  generate_contact_plan
RUNTIME DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tacbot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#rospack plugins --attrib=plugin controller_interface
#rosservice call controller_manager/list_controller_types