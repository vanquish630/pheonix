# Installation 
## Docker

You can find these installation instructions [here](https://docs.px4.io/master/en/test_and_ci/docker.html).

#### **NOTE** : PX4 containers are currently only supported on Linux


## Installing Docker

#### Install docker using [convenience script](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-convenience-script)

    curl -fsSL get.docker.com -o get-docker.sh
    sudo sh get-docker.sh

#### Steps to use docker without having to use sudo
    
    sudo groupadd docker
    # Add your user to the docker group.
    sudo usermod -aG docker $USER
    # Close the terminal or restart computer to see effects

## Setting up workspace on hosts computer

#### We are setting up the workspace on users computer which will be linked to the container. You could do the same inside the container itself, but you will loose all your data once you remove the container.

    mkdir -p pheonix_project/home
    cd pheonix_project/home
    git clone https://github.com/PX4/PX4-Autopilot.git
    mv PX4-Autopilot px4
    cd px4


## Building docker container

#### Run the container in interactive mode terminal with bash. If the image is not present, it is automatically downloaded form docker hub. Here we are mounting hosts directory (where the catkin workspace and firmware is present) to the container directory using -v command. Changes are bidirectional.

- privilaged : it will enable access to hosts ports such as usb
- v : linking host and docker folders
- it : enable interactive terminal 
- p : port mapping from host to container, format of < host >:< container >
- name : name of container

image location (docker hub) : sohananisetty/pheonix-ros-melodic-px4-gazebo


    # enable access to xhost from the container
    xhost +

    # Run docker and open bash shell

    docker run -it --privileged --env=LOCAL_USER_ID="$(id -u)" -v ~/pheonix_project/home:/home/:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 -p 14556:14556/udp --name=px4container sohananisetty/pheonix-ros-melodic-px4-gazebo bash

#### **Note**: Run the below commands inside the container.
####  Build SITL (run in px4 directory). This is for building quadcopter with gazebo simulator. For other vehicle types or other simulators see the [documentation](https://docs.px4.io/master/en/simulation/). 

    cd home/px4    #This is in container
    make px4_sitl_default gazebo


## Access the container in a new terminal/ start container

    # start the container
    docker start container_name
    # open a new bash shell in this container
    docker exec -it container_name bash

#### If you don't remember container name

    docker ps -a ##list all containers

## Removing a container

#### **Note** : Your local container data will be lost if not linked to hosts computer

    docker rm container_name


# Running the simulation

#### start your docker container if previously stopped. Closing the terminal does not stop the container.

#### See the active containers
    docker ps

#### Navigate into catkin workspace (inside container). 

    mkdir -p home/catkin_ws/src
    cd home/catkin_ws/src
    git clone https://github.com/vanquish630/pheonix.git
    cd ..
    catkin_make
    source devel/setup.bash

#### Your home directory should have this structure :

    home
    --catkin_ws
       --src
         --pheonix
    --px4

#### Add following add the end of bashrc
    px4_dir=/home/px4
    source /opt/ros/melodic/setup.bash
    source /home/catkin_ws/devel/setup.bash
    source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH/home/catkin_ws/src/gazebo_models:/home/catkin_ws/src/pheonix/models
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
    export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH/home/catkin_ws/src/pheonix/worlds


### Run 4 terminals side by side

#### In terminal 1 launch SITL

    roslaunch pheonix gazebo_sitl.launch 


#### In terminal 2 launch mavros

    roslaunch pheonix mavros.launch 

#### In terminal 3 launch service server taking care of movement

    roslaunch pheonix simple_control.launch 

#### In terminal 4 launch custom program

    rosrun pheonix triangle.py


## Problems

#### In case gui error comes or cannot open display error

    #run this in the host machine
    xhost +

#### In case gazebo gets stuck in physics properties

    # download models from git
    git clone https://github.com/osrf/gazebo_models.git
    # copy contents of downloaded directory to gazebo model directory
    cp -r <download_location>/* ~/.gazebo/models 

#### In case px4 or mavlink_sitl package not found

    source ~/.bashrc

#### In case gazebo error 255 comes or if gazebo reloading past session instead of bringing up a fresh session

    killall gzserver


