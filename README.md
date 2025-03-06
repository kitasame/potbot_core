# potbot_core
cd ~/catkin_ws/src

git clone -b personal_space https://github.com/kitasame/potbot_core.git


sudo apt update

sudo apt install ros-$ROS_DISTRO-navigation


cd ~/catkin_ws

catkin build potbot_plugin
