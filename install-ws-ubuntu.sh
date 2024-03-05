#!/bin/bash

BLUE='\033[0;34m'
NC='\033[0m'
set -eo pipefail
SUDO="sudo -H"
ubuntu_version=`lsb_release -rs | sed 's/\.//'`
ROS_DISTRO=noetic

install_common_dependencies()
{
  # install most dependencies via apt-get
  echo -e "${BLUE}Update and upgrade packages${NC}"
  ${SUDO} apt -y update
  ${SUDO} apt -y upgrade

  # OMPL
  echo -e "${BLUE}Installing OMPL dependencies${NC}"
  # We explicitly set the C++ compiler to g++, the default GNU g++ compiler. This is
  # needed because we depend on system-installed libraries built with g++ and linked
  # against libstdc++. In case `c++` corresponds to `clang++`, code will not build, even
  # if we would pass the flag `-stdlib=libstdc++` to `clang++`.
  ${SUDO} apt -y install g++ cmake pkg-config libboost-serialization-dev libboost-filesystem-dev libboost-system-dev libboost-program-options-dev libboost-test-dev libeigen3-dev libode-dev wget libyaml-cpp-dev
  export CXX=g++
  export MAKEFLAGS="-j `nproc`"

  # libfranka
  echo -e "${BLUE}Installing libfranka dependencies${NC}"
  ${SUDO} apt -y install build-essential cmake git libpoco-dev libeigen3-dev

  # moveit
  echo -e "${BLUE}Installing moveit dependencies${NC}"
  ${SUDO} apt install python3-wstool python3-catkin-tools clang-format-10 python3-rosdep

  # ros
  ${SUDO} apt -y install ros-${ROS_DISTRO}-rosparam-shortcuts
  ${SUDO} apt -y install ros-${ROS_DISTRO}-object-recognition-msgs
  ${SUDO} apt -y install ros-${ROS_DISTRO}-octomap-msgs
  ${SUDO} apt -y install ros-${ROS_DISTRO}-combined-robot-hw
  ${SUDO} apt -y install ros-${ROS_DISTRO}-eigen-stl-containers
  ${SUDO} apt -y install ros-${ROS_DISTRO}-geometric-shapes
  ${SUDO} apt -y install ros-${ROS_DISTRO}-srdfdom
  ${SUDO} apt -y install ros-${ROS_DISTRO}-pybind11-catkin
  ${SUDO} apt -y install ros-${ROS_DISTRO}-warehouse-ros
  ${SUDO} apt -y install ros-${ROS_DISTRO}-eigenpy
  ${SUDO} apt -y install ros-${ROS_DISTRO}-rviz-visual-tools
  ${SUDO} apt -y install ros-${ROS_DISTRO}-moveit-visual-tools
  ${SUDO} apt -y install ros-${ROS_DISTRO}-turtlesim
  ${SUDO} apt -y install ros-${ROS_DISTRO}-trac-ik-lib
  ${SUDO} apt -y install ros-${ROS_DISTRO}-nlopt
  ${SUDO} apt -y install ros-${ROS_DISTRO}-controller-interface
  ${SUDO} apt -y install ros-${ROS_DISTRO}-controller-manager
  ${SUDO} apt -y install ros-${ROS_DISTRO}-tf-conversions
  ${SUDO} apt -y install ros-${ROS_DISTRO}-gazebo-ros-control
} 


install_moveit(){
  wstool init src
  wstool merge -t src src/moveit/moveit.rosinstall
  wstool update -t src
  rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
  cd src/moveit
  git checkout ${ROS_DISTRO}-devel
  cd ../../
  git submodule update --recursive
}

build_libfranka()
{
  echo -e "${BLUE}Build and install libfranka${NC}"
  mkdir -p src/libfranka/build
  cd src/libfranka
  git fetch --all --tags
  git checkout 0.10.0
  git submodule update --recursive
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
  cmake --build .
  cd ../../../
}

build_ompl()
{
  echo -e "${BLUE}Build and install OMPL${NC}"
  mkdir -p src/omplapp/ompl/build/Release
  cd src/omplapp/ompl/build/Release
  cmake ../..
  make

  # sub with checkinstall here
  # ${SUDO} make install

  cd ../../../../../
}

build_omplapp()
{
  echo -e "${BLUE}Build and install OMPL.app${NC}"
  mkdir -p src/omplapp/build/Release
  cd src/omplapp/build/Release
  cmake ../..
  make
 
  # sub with checkinstall here
  #${SUDO} make install
 
  cd ../../../../
}

build_ruckig(){
  echo -e "${BLUE}Build and install ruckig${NC}"
  mkdir -p src/ruckig/build
  cd src/ruckig/build/
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make

  # sub with checkinstall here
  # ${SUDO} make install

  cd ../../../
}

set_upstream_branches(){
  #currenlty not working with the build script
  cd src/libfranka
  if git ls-remote --exit-code upstream; then
    echo -e "${BLUE}libfranka upstream already eists${NC}"
  else
    git remote add upstream git@github.com:frankaemika/libfranka.git
  fi
  cd ../../

  cd src/franka_ros
  if git ls-remote --exit-code upstream; then
    echo -e "${BLUE}franka_ros upstream already exists${NC}"
  else
    git remote add upstream git@github.com:frankaemika/franka_ros.git
  fi
  cd ../../

  cd src/omplapp
  if git ls-remote --exit-code upstream; then
    echo -e "${BLUE}omplapp upstream already exists${NC}"
  else
    git remote add upstream git@github.com:ompl/omplapp.git
  fi

  cd ompl
  if git ls-remote --exit-code upstream; then
    echo -e "${BLUE}ompl upstream already exists${NC}"
  else
    git remote add upstream git@github.com:ompl/ompl.git
  fi
  cd ../../../

  cd src/moveit
  if git ls-remote --exit-code upstream; then
    echo -e "${BLUE}moveit upstream already exists${NC}"
  else
    git remote add upstream git@github.com:ros-planning/moveit.git
  fi
  cd ../../
}

pull_origin(){
  git submodule foreach -q --recursive 'git pull'
  cd src/tacbot/
  git pull
  cd ../../
}

fetch_and_merge(){
  #incomplete function, not ready for use
  BRANCH="$(git config -f $toplevel/.gitmodules submodule.$name.branch)";

  if git ls-remote --exit-code upstream; then
    git fetch upstream
    git merge upstream/${BRANCH}
  fi
}

installer(){
  git submodule foreach -q --recursive 'branch="$(git config -f $toplevel/.gitmodules submodule.$name.branch)"; git checkout $branch'
  git submodule update --init --recursive --remote
  install_common_dependencies
  install_moveit
  build_ruckig
  build_libfranka
  # # build_ompl #omplapp already build ompl
  build_omplapp
  git submodule update --recursive
  # set_upstream_branches
}

builder(){
  catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$(pwd)/src/libfranka/build
  # catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/umbreon/libfranka/build
}

helper()
{
   # Display Help
   echo "Script used to help you build this repository."
   echo
   echo "Usage: ./install-ws-ubuntu [-h|i|b|s]"
   echo "Options:"
   echo "h     Help: Display this helper text."
   echo "i     Install: Installs the modules and submodules from their remote repositories. Only needs to be called once unless an error occurs."
   echo "b     Build: Builds the newly installed packages. Can be used as often as necessary until build success."
   echo "s     Sync: Pull from remote and upstream branches then merge the new content to the main noetic branch."
   echo
}

no_args="true"
while getopts ":hibs" option; do
  case $option in
    h) # display Help
      helper
      exit;;
    i) # install packages
      installer
      exit;;
    b) # build packages
      builder
      exit;;
    s) # sync with upstream
      pull_origin
      exit;;
    \?) # invalid option
      echo "Error: Invalid option"
      helper
      exit;;
  esac
  no_args="false"
done

[[ "$no_args" == "true" ]] && { helper; exit 1; }