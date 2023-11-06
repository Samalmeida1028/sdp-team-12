sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop

sudo apt install ros-dev-tools

sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-nav2-simple-commander

sudo apt install ros-humble-slam-toolbox

sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro

sudo apt-get install gedit

sudo apt install ros-humble-gazebo-ros-pkgs

sudo apt-get install ros-humble-rqt-robot-steering

sudo apt install ros-humble-robot-localization
