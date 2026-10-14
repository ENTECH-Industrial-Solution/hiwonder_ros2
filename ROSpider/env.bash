# Environment for this workspace's simulation. Source it in every terminal, or once from ~/.bashrc:
#   echo "source ~/hiwonder_ros2/ROSpider/env.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
ws="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
if [ -f "$ws/install/local_setup.bash" ]; then
    source "$ws/install/local_setup.bash"
else
    echo "env.bash: $ws is not built yet (cd $ws && colcon build --symlink-install)" >&2
fi
unset ws
export need_compile=True
# ROS and Gazebo stay on this machine: sims on one network otherwise share /clock and ignore teleop
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1
