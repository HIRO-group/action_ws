#!/bin/bash

BLUE='\033[0;34m'
NC='\033[0m'
echo -e "${BLUE}Start!${NC}"

set -eo pipefail

SUDO="sudo -H"

ubuntu_version=`lsb_release -rs | sed 's/\.//'`

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
  git remote add upstream git@github.com:frankaemika/libfranka.git
  cd ../../

  cd src/franka_ros
  git remote add upstream git@github.com:frankaemika/franka_ros.git
  cd ../../

  cd src/omplapp
  git remote add upstream git@github.com:ompl/omplapp.git

  cd ompl
  git remote add upstream git@github.com:ompl/ompl.git
  cd ../../../
}

set_upstream_branches

git submodule foreach -q --recursive 'branch="$(git config -f $toplevel/.gitmodules submodule.$name.branch)"; git checkout $branch'

git submodule update --init --recursive --remote

install_common_dependencies
install_libfranka
install_ompl
install_omplapp

catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$(pwd)/src/libfranka/build