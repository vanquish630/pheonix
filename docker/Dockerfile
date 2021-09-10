FROM px4io/px4-dev-ros-melodic:2021-08-18
LABEL Description="ROS-Melodic-PX4"

ENV DEBIAN_FRONTEND noninteractive 
RUN apt-get update && apt-get install -y \
libpcl1 \
ros-melodic-octomap-* \
ros-melodic-image-view \
ros-melodic-stereo-image-proc \
ros-melodic-rqt \
ros-melodic-rqt-common-plugins \
gedit \
psmisc \
apt-get clean && rm -rf /var/lib/apt/lists/*
RUN makedir 




# Configure ROS
RUN rosdep init && rosdep update 
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

RUN mkdir -p /src/
WORKDIR /src/
RUN git clone https://github.com/PX4/PX4-Autopilot.git && mv PX4-Autopilot px4
WORKDIR px4
RUN make px4_sitl_default gazebo
WORKDIR /src/
RUN mkdir - p catkin_ws/src
CMD catkin_make
RUN echo "source /src/catkin_ws/devel/setup.bash" >> ~/.bashrc

