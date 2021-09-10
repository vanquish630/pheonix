FROM px4io/px4-dev-ros-melodic:2021-08-18
LABEL Description="ROS-Melodic-PX4"

ENV DEBIAN_FRONTEND noninteractive 
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
ros-melodic-octomap-* \
ros-melodic-image-view \
ros-melodic-stereo-image-proc \
ros-melodic-rqt \
ros-melodic-rqt-common-plugins \
gedit \
psmisc \
&& apt-get clean && rm -rf /var/lib/apt/lists/*
# Configure ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

