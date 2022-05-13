#!/bin/bash

set -e

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

apt-get update -qq && apt-get install -y -q curl gnupg sudo lsb-release # for docker

# set non interactive tzdata https://stackoverflow.com/questions/8671308/non-interactive-method-for-dpkg-reconfigure-tzdata
# set DEBIAN_FRONTEND=noninteractive
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

# Install system dependencies, namely ROS.
#before_install:
travis_time_start setup.before_install

# Install ROS
sudo sh -c "echo \"deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update -qq
# Install ROS
sudo apt-get install -y python-catkin-pkg python-catkin-tools python-rosdep python-wstool ros-$ROS_DISTRO-catkin
source /opt/ros/$ROS_DISTRO/setup.bash
# Setup for rosdep
sudo rosdep init
# use snapshot of rosdep list
# https://github.com/ros/rosdistro/pull/31570#issuecomment-1000497517
if [[ "$ROS_DISTRO" =~ "hydro"|"indigo"|"jade"|"kinetic"|"lunar" ]]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    sudo curl -s https://gist.githubusercontent.com/cottsay/b27a46e53b8f7453bf9ff637d32ea283/raw/476b3714bb90cfbc6b8b9d068162fc6408fa7f76/30-xenial.list -o /etc/ros/rosdep/sources.list.d/30-xenial.list
fi
rosdep update --include-eol-distros

travis_time_end


# Create a catkin workspace with the package under test.
#install:
travis_time_start setup.install

mkdir -p ~/catkin_ws/src

# Add the package under test to the workspace.
cd ~/catkin_ws/src
ln -s $CI_SOURCE_PATH . # Link the repo we are testing to the new workspace

travis_time_end

# Install all dependencies, using wstool and rosdep.
# wstool looks for a ROSINSTALL_FILE defined in before_install.
#before_script:
travis_time_start setup.before_script

# source dependencies: install using wstool.
cd ~/catkin_ws/src
wstool init

# package depdencies: install using rosdep.
cd ~/catkin_ws
rosdep install -y -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO
travis_time_end

# Compile and test.
#script:
travis_time_start setup.script
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin_make_isolated
rm -fr build* devel* install*
(cd src/openrave_planning; git clean -xfd)
catkin_make_isolated --install
travis_time_end
