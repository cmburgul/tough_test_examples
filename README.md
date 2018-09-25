### tough_test_examples

#### These are the detail explaination for tough_test_examples package   

These package contains the test examples which can be used to move each part of the atlas seperately. This is good for beginners to start with each line is explained line by line and the necessary data is also provided which makes the code easier to understand. 

#### 1. Chest Navigation Example
 The code file is located in __tough_test_examples/src/chest_navigation_example__

 In this example code we will write a ros node to move the chest base such that it's roll=0, pitch=10, and yaw=30 degrees in 5 seconds by default. We can also give arguments as follows 
 ```
 rosrun tough_test_examples chest_navigation_example <arg1 arg2 arg3> 
```
arg1 = roll
arg2 = pitch
arg3 = yaw

 The input arguments are given in degrees. 
  
 To start, let's include the header file for ChestControllerInterface.
```
#include <tough_controller_interface/chest_control_interface.h>
```
 Now that we have access to the functions to control chest, we should initialize a ChestControllerInterface object in a ros node to call those functions. This needs ros::NodeHandle as a constructor argument.
 
```
#include <tough_controller_interface/chest_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of ChestControlInterface
    ChestControlInterface chestTraj(nh);
```
Now we can call the controlChest function to move the chest in required orientation.

```
float roll = 0.0f;
float pitch = 10.0f;
float yaw = 30.0f;
float duration = 5.0f; 

// change the chest orientation. This is a non-blocking call.
chestTraj.controlChest(roll, pitch, yaw, duration);
```
Putting it all together, we have the following code that moves the robot chest to the specified orientation.

#### Code:
```
#include <tough_controller_interface/chest_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of ChestControlInterface
    ChestControlInterface chestTraj(nh);
    float roll = 0;
    float pitch = 10;
    float yaw = 30;
    float duration = 5.0f; 

    // change the chest orientation. This is a non-blocking call.
    chestTraj.controlChest(roll, pitch, yaw, duration);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
```
