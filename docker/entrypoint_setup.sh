#! /bin/bash

mkdir -p /home/catkin_ws/src
cd /home/catkin_ws/src
git clone https://github.com/vanquish630/pheonix.git
cd ..
catkin_make
echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd ..
git clone https://github.com/PX4/PX4-Autopilot.git && mv PX4-Autopilot px4
# WORKDIR px4
# RUN no_sim=1 make px4_sitl_default gazebo && exit
# WORKDIR /src/
# RUN mkdir - p /catkin_ws/src
# CMD catkin_make

### modifying the bashrc file
echo "$px4_dir /home/px4" >> ~/.bashrc
echo "source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH/home/catkin_ws/src/pheonix/models" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH/usr/lib/x86_64-linux-gnu/gazebo-9/plugins" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH/home/catkin_ws/src/pheonix/worlds" >> ~/.bashrc


# Run CMD from Docker
"$@"