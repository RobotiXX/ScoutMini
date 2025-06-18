# Assume this repo is already cloned to $HOME directory

### Remove a conflicting package
sudo apt remove brltty

### Install ZED SDK
cd $HOME
wget --content-disposition https://download.stereolabs.com/zedsdk/4.2/l4t36.4/jetsons
chmod +x ZED_SDK_Tegra_L4T36.4_v4.2.5.zstd.run
./ZED_SDK_Tegra_L4T36.4_v4.2.5.zstd.run -- silent

### Install ROS 2
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init
rosdep update

# Install dependencies
sudo apt-get install libqt5serialport5-dev # for witmotion IMU
sudo apt-get install build-essential git cmake libasio-dev can-utils # for Scout ugv_sdk
sudo pip install evdev

# Copy udev rules 
cd $HOME/socnav_data_collection
sudo cp -r etc/udev/rules.d/* /etc/udev/rules.d/

# For PS4 controller package
cd $HOME/socnav_data_collection/ros2_ws/src/ds4drv
mkdir -p ~/.local/lib/python3.10/site-packages
python3 setup.py install --prefix ~/.local
# sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/ # already did this
sudo udevadm control --reload-rules
sudo udevadm trigger

# For ZED ROS 2 Wrapper
cd $HOME/socnav_data_collection/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
echo source $(pwd)/install/_setup.bash >> ~/.bashrc

echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

source ~/.bashrc
