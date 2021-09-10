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

    mkdir src
    cd src
    git clone https://github.com/PX4/PX4-Autopilot.git
    mv PX4-Autopilot px4
    cd px4


## Building docker container

#### Run the container in interactive mode terminal with bash. If the image is not present, it is automatically downloaded form docker hub. Here we are mounting hosts directory (where the catkin workspace and firmware is present) to the container directory using -v command. Changes are bidirectional.

- privilaged : it will acess to hosts ports such as usb
- v : linking host and docker
- it : enable interactive terminal 
- p : port mapping from host to container
- name : name of container

image location (docker hub) : px4io/px4-dev-ros-melodic:2021-08-18


    # enable access to xhost from the container
    xhost +

    # Run docker and open bash shell

    docker run -it --privileged --env=LOCAL_USER_ID="$(id -u)" -v ~/px4/src:/src/:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 -p 14556:14556/udp --name=pxcontainer px4io/px4-dev-ros-melodic:2021-08-18 bash

#### **Note**: Run the below commands inside the container.
####  Build SITL (run in px4 directory). This is for building quadcopter with gazebo simulator. For other vehicle types or other simulators see the [documentation](https://docs.px4.io/master/en/simulation/). 

    cd src/px4    #This is <container_src>
    make px4_sitl_default gazebo


## Access the container in a new terminal/ start container

    # start the container
    docker start container_name
    # open a new bash shell in this container
    docker exec -it container_name bash

#### If you don't remember container name

    docker ps -a

## Removing a container

#### **Note** : Your local container data will be lost if not linked to hosts computer

    docker rm container_name


# Running the simulation

#### start your docker container if previously stopped. Closing the terminal does not stop the container.

#### See the active containers
    docker ps

#### navigate into catkin workspace 

    cd src/catkin_ws/src
    git clone 
    cd ..
    catkin_make
    source devel/setup.bash



