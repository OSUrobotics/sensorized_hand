colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

source /opt/ros/humble/setup.bash
. install/setup.bash
pip install setuptools==58.2.0

ros2 run tof talker 

colcon build --packages-select tof

ros2 interface show hand_msgs/msg/Tofzone
colcon build --symlink-install

colcon clean workspace

i2cdetect -y 1
7 bit to 8 bit i2c representation....

ros2 interface show 

rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install

rate.sleep is blocking in motor implementation - temporarily just using time sleep (search for a better implementation?) May also not be an issue as before ros spin

# Todo
Possibly timestamp/header all of my custom msg types?


<!-- vcgencmd get_throttled -->
vcgencmd measure_temp