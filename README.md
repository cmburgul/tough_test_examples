### tough_test_examples

#### These are the detail explaination for tough_test_examples package   

These package contains the test examples which can be used to move each part of the atlas seperately. This is good for beginners to start with each line is explained line by line and the necessary data is also provided which makes the code easier to understand. 

#### 1. Testing Chest movements

#### Code:
```
#include <tough_controller_interface/chest_control_interface.h>
#include <tf/transform_listener.h>

#define TO_RADIANS M_PI / 180.0 //goes probably in utils which stores similar math operation parameters

ChestControlInterface::ChestControlInterface(ros::NodeHandle nh):ToughControllerInterface(nh)
{

    chestTrajPublisher_ =
            nh_.advertise<ihmc_msgs::ChestTrajectoryRosMessage>(control_topic_prefix_ +"/chest_trajectory",1,true);
}

ChestControlInterface::~ChestControlInterface()
{
}

void ChestControlInterface::controlChest(float roll , float pitch , float yaw, float time, int execution_mode)
{

    roll  =  roll*TO_RADIANS;
    pitch = pitch*TO_RADIANS;
    yaw   =   yaw*TO_RADIANS;

    tf::Quaternion quatInPelvisFrame;
    quatInPelvisFrame.setRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(quatInPelvisFrame, quat);

    controlChest(quat, time, execution_mode);
}

void ChestControlInterface::controlChest(geometry_msgs::Quaternion quat, float time, int execution_mode)
{
    ihmc_msgs::ChestTrajectoryRosMessage msg;
    ihmc_msgs::SO3TrajectoryPointRosMessage data;

    data.time = time;

    data.orientation = quat;
    msg.unique_id = ChestControlInterface::id_++;
    msg.execution_mode = execution_mode;


    msg.taskspace_trajectory_points.push_back(data);

    // publish the message
    chestTrajPublisher_.publish(msg);

}

void ChestControlInterface::getChestOrientation(geometry_msgs::Quaternion &orientation)
{
        geometry_msgs::Pose chest_pose;
        state_informer_->getCurrentPose(rd_->getTorsoFrame(), chest_pose, rd_->getPelvisFrame());
        orientation = chest_pose.orientation;
}
```
#### Explaination : 

   These test example takes 3 arguments: roll, pitch, & yaw
   Arguments is taken in degrees and converted to radians

```
#include <tough_controller_interface/chest_control_interface.h>
#include <tf/transform_listener.h>
```
