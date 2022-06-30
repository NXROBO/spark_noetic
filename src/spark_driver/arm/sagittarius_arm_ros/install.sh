
#检查系统要求
check_sys(){
	ubuntu_version=$(lsb_release -r --short)
        if [[ "${ubuntu_version}" == "16.04" ]]; then
                ROS_Ver="kinetic"
        elif [[ "${ubuntu_version}" == "18.04" ]]; then
                ROS_Ver="melodic"
        elif [[ "${ubuntu_version}" == "20.04" ]]; then
                ROS_Ver="noetic"
        else
                echo -e "${Error} SPARK暂不支持当前系统 ${OSDescription} !" && exit 1
        fi
}
BASEPATH=$(cd `dirname $0`; pwd)
echo "sudo cp $BASEPATH/sdk_sagittarius_arm/rules/sagittarius-usb-serial.rules /etc/udev/rules.d/"
sudo cp $BASEPATH/sdk_sagittarius_arm/rules/sagittarius-usb-serial.rules /etc/udev/rules.d/
sudo udevadm trigger
check_sys
sudo apt -y install ros-${ROS_Ver}-dynamixel-workbench-toolbox ros-${ROS_Ver}-moveit-msgs ros-${ROS_Ver}-moveit-ros ros-${ROS_Ver}-ompl ros-${ROS_Ver}-moveit-planners-ompl ros-${ROS_Ver}-moveit-simple-controller-manager ros-${ROS_Ver}-joint-state-publisher-gui ros-${ROS_Ver}-moveit-commander ros-${ROS_Ver}-trac-ik-kinematics-plugin ros-${ROS_Ver}-moveit-setup-assistant ros-${ROS_Ver}-moveit-fake-controller-manager ros-${ROS_Ver}-moveit-visual-tools ros-${ROS_Ver}-joy  ros-${ROS_Ver}-joint-trajectory-controller ros-${ROS_Ver}-joint-state-controller ros-${ROS_Ver}-gazebo-ros-control ros-${ROS_Ver}-effort-controllers ros-${ROS_Ver}-chomp-motion-planner ros-${ROS_Ver}-moveit-planners-chomp ros-${ROS_Ver}-pilz-industrial-motion-planner

if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    echo "Installing librealsense2..."
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -sc) main" -u
    if [ $ubuntu_version == "16.04" ]; then
      version="2.48.0-0~realsense0.4974"
    elif [ $ubuntu_version == "18.04" ]; then
      version="2.48.0-0~realsense0.4975"
    elif [ $ubuntu_version == "20.04" ]; then
      version="2.48.0-0~realsense0.4976"
    fi

    sudo apt -y install librealsense2-udev-rules=${version}
    sudo apt -y install librealsense2-dkms
    sudo apt -y install librealsense2=${version}
    sudo apt -y install librealsense2-gl=${version}
    sudo apt -y install librealsense2-gl-dev=${version}
    sudo apt -y install librealsense2-gl-dbg=${version}
    sudo apt -y install librealsense2-net=${version}
    sudo apt -y install librealsense2-net-dev=${version}
    sudo apt -y install librealsense2-net-dbg=${version}
    sudo apt -y install librealsense2-utils=${version}
    sudo apt -y install librealsense2-dev=${version}
    sudo apt -y install librealsense2-dbg=${version}
    sudo apt-mark hold librealsense2*
    sudo apt -y install ros-${ROS_Ver}-ddynamic-reconfigure
else
    echo "librealsense2 already installed!"
fi

echo "sudo cp $BASEPATH/sdk_sagittarius_arm/rules/usb_cam.rules /etc/udev/rules.d/"
sudo cp $BASEPATH/sdk_sagittarius_arm/rules/usb_cam.rules /etc/udev/rules.d/
sudo udevadm trigger
check_sys
