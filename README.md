# ROS 2 Publisher-Subscriber Package (C++)

This ROS 2 package demonstrates a simple publisher-subscriber application using C++.

## Overview

This package contains two nodes:

1. **Publisher Node**: Publishes a "Hello, Turtlesquad! message to the "chatter" topic at a fixed rate.
2. **Subscriber Node**: Subscribes to the "chatter" topic and prints received messages to the console.

## Build

To build the package, follow these steps:

0. Install a few dependencies and clone the repository
```
apt install cppcheck
git clone https://github.com/1412kauti/beginner_tutorials.git
cd beginner_tutorials
cp cpp_pubsub ~/path/to/workspace
```
1. Copy the package `cpp_pubsub` to the `src` folder of your existing ROS2 workspace or create a new workspace using:
```zsh
mkdir -p test_ws/src
cd test_ws/src
# copy the cpp_pubsub folder here  
cd ../../
colcon build --select-packages cpp_pubsub
```
2. Install Dependencies if any:
```zsh
rosdep install -i --from-path src --rosdistro humble -y   
```

4. Source the workspace depending on the workspace:
```zsh
source install/setup.bash #(bash)
source install/setup.zsh #(zsh)
```

5. Run the publisher and subscriber

In Terminal 1:
```zsh
cd ~/path/to/test_ws
source install/setup.zsh #Follow step 4 
ros2 run cpp_pubsub talker
```

In Terminal 2:
```zsh
cd ~/path/to/test_ws
source install/setup.zsh #Follow step 4 
ros2 run cpp_pubsub listener
```
## Linting
1. cpplint
```sh
find src/ -name "*.cpp" -o -name "*.h" | xargs cpplint 2>&1 | tee cpplint_output.txt
```

2. cppcheck
```sh
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem src/ >> cppcheck_out.txt
```
