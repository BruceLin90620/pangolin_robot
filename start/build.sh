echo "->>>>>>>>>>>>>>Install ROS2 humble<<<<<<<<<<<<<-"

echo "" 
echo "->>>>>>>>>>>>>>Set locale<<<<<<<<<<<<<-"
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings


echo "" 
echo "->>>>>>>>>>>>>>Setup Source<<<<<<<<<<<<<-"
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


echo "" 
echo "->>>>>>>>>>>>>>Install ROS2 packages<<<<<<<<<<<<<-"
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc


echo "" 
echo "->>>>>>>>>>>>>>build pangolin workspace<<<<<<<<<<<<<-"
mkdir pangolin_ws && cd pangolin_ws
git clone https://github.com/JaythanCheng1210/ROS2_Pangolin.git
sudo apt install python3-pip
sudo pip3 install pigpio
sudo pip3 install RPi.GPIO
sudo pip3 install smbus2
pip3 install setuptools==58.2.0
sudo apt install ros-humble-joy && sudo apt install ros-humble-teleop-twist-joy
cd pangolin_ws
colcon build --symlink-install

echo "" 
echo "->>>>>>>>>>>>>>DynamixelSDK setup<<<<<<<<<<<<<-"
cd 
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd ~/DynamixelSDK/python/ && sudo python3 setup.py install



systemctl mask systemd-networkd-wait-online.service

chmod 777 /dev/i2c-1

