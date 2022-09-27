#!/bin/bash

BLUE='\033[0;34m'
NC='\033[0m'
echo -e "${BLUE}Start!${NC}"

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
    sudo apt install python3-wstool python3-catkin-tools clang-format-10 python3-rosdep
    sudo apt install ros-${ROS_DISTRO}-rosparam-shortcuts
}

install_libfranka()
{
  echo -e "${BLUE}Install libfranka${NC}"
  mkdir -p src/libfranka/build
  cd src/libfranka/build
  cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
  cmake --build .
  cd ../../../
}

install_ompl()
{
  echo -e "${BLUE}Install OMPL${NC}"
  mkdir -p src/omplapp/ompl/build/Release
  cd src/omplapp/ompl/build/Release
  cmake ../..
  make
  ${SUDO} make install
  cd ../../../../../
}

install_omplapp()
{
  echo -e "${BLUE}Install OMPL.app${NC}"
  mkdir -p src/omplapp/build/Release
  cd src/omplapp/build/Release
  cmake ../..
  make
  ${SUDO} make install
  cd ../../../../
}

set_upstream_branches(){
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

set_upstream_branches

git submodule foreach -q --recursive 'branch="$(git config -f $toplevel/.gitmodules submodule.$name.branch)"; git checkout $branch'

git submodule update --init --recursive --remote

install_common_dependencies
install_libfranka
install_ompl
install_omplapp

catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$(pwd)/src/libfranka/build