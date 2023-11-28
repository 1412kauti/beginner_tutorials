# Week2
This ROS2 package demonstrates the use of Services
## Overview:
This repository consists of four packages:
1. **cpp_pubsub**: 
	a. **Publisher Node**: Publishes an Integer ato the "chatter" topic at a fixed rate
	b. **Subscriber Node**: Subscribes to the "chatter" topic and prints the recieved message to the console.
2. **tutorial_interfaces**:
	a. 1. **Num.msg**: A custom message of type `int64`
		2. **Sphere.msg**: A custom message of type `float64`
	b. 1. **AddThreeInts.srv**: A custom service to add three discrete Integers of type `int64`
		2. **AddTwoInts.srv**: A custom service to add two discrete Integeres of type `int64`
3. **cpp_srvcli**:
	a. **Three Integers Server**: A Service server to add Three Integers
	b. **Three Integers Client**: A Service Client to add Three Integers
	c. **Two Integers Server**: A Service server to add Two Integers
	d. **Two Integers Client**: A Service server to add Two Integers
4. **cpp_parameters**:
	a. **Parameters Node**: A Publisher that would publish any message that was provided as a parameter 

## Build
To build all the packages, follow the steps:

```zsh
sudo apt install cppcheck cpplint
cd </path/to/ros2_workspace>
cd src/
git clone https://github.com/1412kauti/beginner_tutorials.git
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.zsh # setup.bash or setup.sh 
```

## Run
### Service to Run Two Integers:

Terminal 1:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh 
ros2 run cpp_srvcli server2
```

Terminal 2:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh 
ros2 run cpp_srvcli client2
```

### Service to Run Three Integers:

Terminal 1:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh 
ros2 run cpp_srvcli server3
```

Terminal 2:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh 
ros2 run cpp_srvcli client3
```

### Custom Messages

Terminal 1:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh 
ros2 run cpp_pubsub talker
```

Terminal 2:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh 
ros2 run cpp_pubsub listener
```

### Parameters

Terminal 1:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh
ros2 run cpp_parameters minimal_param_node
```

Terminal 2:
```zsh
cd </path/to/ros2_workspace>
source install/setup.zsh # setup.bash or setup.sh
ros2 param set /minimal_param_node my_parameter <custom_Message>
```
## Linting

```zsh
chmod +x automation_scripts/cpplint_script.sh
./automation_scripts/cpplint_script.sh
```

## CPPCheck
```zsh
chmod +x automation_scripts/cppcheck_script.sh
./automation_scripts/cppcheck_script.sh
```