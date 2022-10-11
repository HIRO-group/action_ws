#!/bin/bash
set -eo pipefail

SUDO="sudo -H"

fetch_and_merge(){
  BRANCH="$(git config -f $toplevel/.gitmodules submodule.$name.branch)";

  if git ls-remote --exit-code upstream; then
    git fetch upstream
    git merge upstream/$(BRANCH)
  else
    git remote add upstream git@github.com:ompl/omplapp.git
  fi
}

pull_origin(){
  git submodule foreach -q --recursive 'git pull'
}

pull_origin