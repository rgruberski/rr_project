# Reflect Robotics Course Project

Simple project from the Robot Operating System 2 training by <https://reflectrobotics.com/>

## How to run it

Run the following commands in your terminal. The code was checked on the Linux Ubuntu system only.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/reflect_ws/src
cd ~/reflect_ws/src
git clone https://github.com/rgruberski/rr_project.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rr_project
source install/setup.bash
ros2 launch rr_project rr_project.launch.py
```

## How it works

You can check how it works in the video below. Just click it and watch :)

[![Watch the video](https://i9.ytimg.com/vi/qUB72aAk0vM/mqdefault.jpg?sqp=CMy0paUG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGCYgKCh_MA8=&rs=AOn4CLDVxJv3WHbJvwMDETYIaAxxBSKVAQ)](https://youtu.be/qUB72aAk0vM)
