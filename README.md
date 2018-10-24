### tough_test_examples

#### These are the detail explaination for tough_test_examples package   

These package contains the test examples which can be used to move each part of the atlas seperately. This is good for beginners to start with each line is explained line by line and the necessary data is also provided which makes the code easier to understand. 

#### 1. Chest Navigation Example
 The code file is located in __tough_test_examples/src/chest_navigation_example__

 In this example code we will write a ros node to move the chest base such that it's roll=0, pitch=10, and yaw=30 degrees in 5 seconds by default. 
  
 To start, let's include the header file for ChestControllerInterface, so that we can build Chest Control Interface objects.

```cpp
#include <tough_controller_interface/chest_control_interface.h>
```
 Now that we have access to the functions to control chest, we should initialize a ChestControllerInterface object in a ros node to call those functions. This needs ros::NodeHandle as a constructor argument.
 
```cpp
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

```cpp
float roll = 0.0f;
float pitch = 10.0f;
float yaw = 30.0f;
float duration = 5.0f; 

// change the chest orientation. This is a non-blocking call.
chestTraj.controlChest(roll, pitch, yaw, duration);
```
Putting it all together, we have the following code that moves the robot chest to the specified orientation.

#### Code:
```cpp
#include <tough_controller_interface/chest_control_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

void help()
{
    std::string help_str = "Usage:\n Give Chest Orientation values in degrees : [Roll] [Pitch] [Yaw] \n For Example: rosrun tough_test_examples 10 0 0";
    std::cout << help_str << std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "chest_navigation_example");                //Initailizing node
  ros::NodeHandle nh;                                               //Creating Nodehandle object
  ROS_INFO("Moving the chest");
  geometry_msgs::Quaternion chest_orientation_before_motion;        //Creating Quaternion object
  geometry_msgs::Quaternion chest_orientation_after_motion;         //Creating Quaternion object
  ChestControlInterface chestTraj(nh);                              //Creating ChestControlInterface object
  tf::StampedTransform utorso_tf;
  tf::TransformListener listener;

  if(argc != 4)
  {
    help();
    return 0;
  }
  else
  {
    float roll  = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw   = std::atof(argv[3]);

    //chestTraj.getChestOrientation(chest_orientation_before_motion);
    listener.lookupTransform("/world", "/utorso",ros::Time(0), utorso_tf);

    chest_orientation_before_motion.x = utorso_tf.getOrigin().getX();
    chest_orientation_before_motion.y = utorso_tf.getOrigin().getY();
    chest_orientation_before_motion.z = utorso_tf.getOrigin().getZ();

    ROS_INFO("Chest Orientation before Motion is : \n roll:   %f \n pitch: %f \n yaw:    %f",
             (float)chest_orientation_before_motion.x,
             (float)chest_orientation_before_motion.y,
             (float)chest_orientation_before_motion.z);

    ROS_INFO("Moving chest angle to \n roll:  %f \n pitch: %f \n yaw:   %f",
             (float)roll,
             (float)pitch,
             (float)yaw);
    //chestTraj.getChestOrientation(chest_orientation_before_motion);
    chestTraj.controlChest(roll, pitch, yaw);
  }

  ros::spinOnce();

  // wait for motion to complete
  ros::Duration(5).sleep();

  // populate chest orientation
  listener.lookupTransform("/world", "/utorso",ros::Time(0), utorso_tf);

  chest_orientation_after_motion.x = utorso_tf.getOrigin().getX();
  chest_orientation_after_motion.y = utorso_tf.getOrigin().getY();
  chest_orientation_after_motion.z = utorso_tf.getOrigin().getZ();

  ROS_INFO("Chest Orientation after Motion is : \n roll:   %f \n pitch: %f \n yaw:    %f",
           (float)chest_orientation_after_motion.x,
           (float)chest_orientation_after_motion.y,
           (float)chest_orientation_after_motion.z);

  ROS_INFO("Motion Successfully Completed");

  return 0;
}
```

### 2. Pelvis Navigation Example
   In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

//Maximum height it can reach is 0.925889
//Minimum height it can reach is 0.5000

void help(){
    std::string help_str = "Usage:\n Give Pelvis Height value : [Height] \n For Example: rosrun tough_test_examples pelvis_navigation_example 1.0";
    std::cout << help_str << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pelvis_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the pelvis");
  PelvisControlInterface pelvisTraj(nh);
  geometry_msgs::Pose msg;
  RobotSide side;

  pelvisTraj.getTaskSpaceState(msg, side);
  ROS_INFO("The height before motion : %f", (float)msg.position.z);

  if(argc != 2)
  {
      help();
      return 0;
  }
  else
  {
    float height = std::atof(argv[1]);
    ROS_INFO("Moving pelvis to height : %f",(float)height);
    pelvisTraj.controlPelvisHeight(height);
    pelvisTraj.getTaskSpaceState(msg, side);
  }
  ros::Duration(5).sleep();
  ROS_INFO("The height after motion : %f", (float)msg.position.z);

  ros::spinOnce();
  // Wait for motion to complete
  ros::Duration(2).sleep();

  ROS_INFO("Motion completed successfully");

  return 0;
}


```
### 3. Reset Robot Example
In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>
#include <tough_controller_interface/pelvis_control_interface.h>
#include <tough_controller_interface/head_control_interface.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_robot");
    ros::NodeHandle nh;

    // initializing objects
    ArmControlInterface armTraj(nh);
    ChestControlInterface chestTraj(nh);
    PelvisControlInterface pelvisTraj(nh);
    HeadControlInterface headTraj(nh);


    //resetting pelvis height to 0.9
    pelvisTraj.controlPelvisHeight(0.9);
    ROS_INFO("Setting Pelvis Height to 0.9");
    ros::Duration(1.5).sleep();

    // resetting chest orientation to roll [2] pitch [2] yaw [2]
    chestTraj.controlChest(2,2,2);
    ROS_INFO("Setting Chest Orientation to 2,2,2");
    ros::Duration(1).sleep();

    // resetting neck orientation to roll [1.5] pitch [0] yaw [0]
    headTraj.moveNeckJoints({{ 1.57f, 0.0f, 0.0f }}, 2.0f);
    ROS_INFO("Setting Neck Orientation to 1.57 0 0");
    ros::Duration(2).sleep();

    armTraj.moveToDefaultPose(RobotSide::LEFT);
    ROS_INFO("Setting Robot Left arm to Default pose");
    ros::Duration(0.3).sleep();
    armTraj.moveToDefaultPose(RobotSide::RIGHT);
    ROS_INFO("Setting Robot Right arm to Default pose");
    ros::Duration(1).sleep();

    return 0;
}


```
### 4. Head Navigation Example
In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>

void help()
{
    std::string help_str = "Usage:\n Give Head Orientation values in degrees : [Roll] [Pitch] [Yaw] \n For Example: rosrun tough_test_examples head_navigation_example 10 0 0";
    std::cout << help_str << std::endl;
}


HeadControlInterface* headTraj;


void headCallBack(std_msgs::Float32MultiArray msg)
{
    if(msg.data.size() != 3)
    {
        return;
    }
    float roll = msg.data[0];
    float pitch = msg.data[1];
    float yaw = msg.data[2];
    ROS_INFO("Roll : %f Pitch : %f Yaw : %f", roll, pitch, yaw);
    headTraj->moveHead(roll, pitch, yaw);
    ros::Duration(0.5).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_head_navigation", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  headTraj = new HeadControlInterface(nh);
  tf::StampedTransform head_tf;
  tf::TransformListener listener;
  geometry_msgs::Quaternion pose;


  ros::Subscriber sub = nh.subscribe("/head_control",10, headCallBack);

  if(argc != 4)
  {
    help();
    return 0;
  }
  else
  {
    ROS_INFO("Moving the head");
    float roll = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw = std::atof(argv[3]);

    ROS_INFO("Moving head to direction : \n roll: %f \n pitch: %f \n yaw: %f",
             (float)roll,
             (float)pitch,
             (float)yaw);
    headTraj->moveHead(roll, pitch, yaw);

    ros::spinOnce();
    ros::Duration(2).sleep();

    listener.lookupTransform("/pelvis", "/head",ros::Time(0), head_tf);
    pose.x = head_tf.getOrigin().getX();
    pose.y = head_tf.getOrigin().getY();
    pose.z = head_tf.getOrigin().getZ();
    ROS_INFO("\n roll: %f \n pitch: %f \n yaw: %f ",
             (float)pose.x,
             (float)pose.y,
             (float)pose.z);

    ROS_INFO("Motion finished");
  }


  return 0;
}
```

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
    std::cout << pelvisInt.getPelvisHeight() << std::endl;
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
```
### 5. Neck Navigation Example
In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
    std::cout << pelvisInt.getPelvisHeight() << std::endl;
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
```
### 6. Wrist Example

In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
    std::cout << pelvisInt.getPelvisHeight() << std::endl;
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
```
### 7. Nudge Local Example
In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
    std::cout << pelvisInt.getPelvisHeight() << std::endl;
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
```

### 8. Arm Navigation Example
In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_arm_navigation");
    ros::NodeHandle nh;


    if (argc != 9) 
    {
        //ROS_INFO("Expected 8 arguments, got " + std::to_string(argc - 1) + ". Exiting.");
	//ROS_INFO()        
	return -1;
    }

    ArmControlInterface armTraj(nh);
    std::vector<double> positions;
    for (int i = 0; i < 7; i ++){
        positions.push_back(std::stof(argv[i+2]));
    }
    std::vector< std::vector<double> > armData;
    armData.push_back(positions);
    RobotSide side = argv[1][0] == '0' ? RobotSide::LEFT : RobotSide::RIGHT;
    std::string side_str = (side == RobotSide::LEFT) ? "left" : "right";

    //ROS_INFO("Moving %s arm to given joint angles",(string)side_str);
//    armData.push_back(positions);
    armTraj.moveArmJoints(side, armData, 2.0f);

    ros::Duration(1).sleep();

    ROS_INFO("Motion complete");
    return 0;
}


```

### 9. Gripper Control Example
In this example, we will write an example node that makes the robot's pelvis height change to its maximum height, 1.05 meters above the left foot frame.

We will start by including the header file of the Pelvis Interface, so that we can build Pelvis Control Interface objects.

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>
```
Now that we have the ability to create a PelvisControlInterface object in our example and we have access to the methods that will actually move our robot, we should create the object. The constructor of the PelvisControlInterface requires a parameter: a NodeHandle object. Therefore, we must declare a NodeHandle, and then initialize, like so:

```cpp
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
```
Now, once we have created the Interface object, we can change the height of the pelvis to 1.05 meters. The robot starts off at around 0.90 meters for pelvis height.

```cpp
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);
```
Once all of that code is put together, the final program should like the following:

```cpp
#include <tough_controller_interface/pelvis_control_interface.h>

int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of PelvisControlInterface - used for actually altering the height of the robot
    PelvisControlInterface pelvisInt(nh);
    std::cout << pelvisInt.getPelvisHeight() << std::endl;
    float height = 1.05;

    // change the pelvis height. This is a non-blocking call.
    pelvisInt.controlPelvisHeight(height);

    // wait for the robot to move
    ros::Duration(2).sleep();
    ROS_INFO("Motion finished");
    return 0;
}
```

