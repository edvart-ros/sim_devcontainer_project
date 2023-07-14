# Start from Ubuntu 20.04 base image
FROM ubuntu:20.04

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# This will make apt-get install non-interactive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Update the apt package list
RUN apt-get update

# Install ROS Noetic
# Instructions taken from: http://wiki.ros.org/noetic/Installation/Ubuntu
RUN apt-get install -y curl gnupg2 lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-noetic.list

RUN apt-get update && \
    apt-get install -y ros-noetic-desktop

# Initialize rosdep
RUN apt-get install -y python3-rosdep && \
    rosdep init && \
    rosdep update

RUN apt install -y gazebo11 libgazebo11-dev

RUN apt install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential

RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash 

# use bash -c to execute the following commands, which need to use source or other bash-specific functionality
RUN /bin/bash -c "source ~/.bashrc"
#RUN /bin/bash -c "nvm install 8"
#RUN /bin/bash -c "source /usr/share/gazebo/setup.sh"

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog
