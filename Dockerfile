#!/usr/bin/env docker

# This is a template for a Dockerfile to build a docker image for your ROS package. 
# The main purpose of this file is to install dependencies for your package.

# FROM ros:noetic-ros-base-focal
# FROM dustynv/ros:noetic-ros-base-l4t-r32.4.4
# FROM ros:melodic-ros-base-bionic       ####<--- TODO: change to your base image
FROM ros:noetic-ros-base-focal

ENV ROS_ROOT=/opt/ros/noetic   
#ENV ROS_ROOT=/opt/ros/melodic          ###<--- TODO: change to your ROS version to mach base image

# Set upp workspace variables
ENV ROS_PACKAGE_NAME=${PACKAGE_NAME}

# Set upp workspace
RUN mkdir -p /ws/src   
WORKDIR /ws

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Package apt dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    git \
    nano \
    cmake \
    python3-pip \
    # ros-noetic-ros-numpy \
    libeigen3-dev \
    libomp-dev \
    libpcl-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-noetic-pcl-ros \
    libomp-dev \
    ros-noetic-tf2-eigen \
    # EXAMPLE: \
    # build-essential \
    # libssl-dev \
    # libffi-dev \
    # python3-setuptools \
    # python3-venv \
    # python3-tk \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# Installing of pip dependencies
# Installing of pip dependencies
# RUN pip3 install --upgrade \
#     && pip3 install \
#     numpy \
#     catkin_tools \
#     rospkg 

#RUN pip3 install \
#     # EXAMPLE: \
#     # torch \
#     # torchvision \
#     # tensorboardX \
#     # opencv-python \
#     # scikit-image \
#     # scikit-learn \


# Optional: Install additional dependencies with script
#COPY scripts/install.sh scripts/
#RUN chmod +x scripts/install.sh && bash .scripts/install.sh

WORKDIR /ws
