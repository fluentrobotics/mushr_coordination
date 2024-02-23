/**
 * Task Allocation test data publisher
 * 
 * Jeeho Ahn
*/


#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mushr_coordination/GoalPoseArray.h>
#include <cmath>

geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
    geometry_msgs::Quaternion quaternion;

    // Convert 2D Euler angles to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    quaternion.w = cy * cr * cp + sy * sr * sp;
    quaternion.x = cy * sr * cp - sy * cr * sp;
    quaternion.y = cy * cr * sp + sy * sr * cp;
    quaternion.z = sy * cr * cp - cy * sr * sp;

    return quaternion;
}

void publishObstacles(ros::Publisher& pub) {

    // Create a PoseArray message for obstacles
    geometry_msgs::PoseArray obstaclesMsg;
    obstaclesMsg.poses.clear();          

    // Publish the PoseArray message for obstacles
    pub.publish(obstaclesMsg);
}

//ex3
void publishInitPose(ros::Publisher& pub, double x_in, double y_in, double th_in) {

    // Create a PoseStamped message for car2's initial pose
    geometry_msgs::PoseStamped initPoseMsg;
    initPoseMsg.header.stamp = ros::Time::now();
    initPoseMsg.header.frame_id = "map";  // Assuming a map frame, adjust as needed

    geometry_msgs::Pose initPose;
    initPose.position.x = x_in;
    initPose.position.y = y_in;
    initPose.orientation = eulerToQuaternion(0,0,th_in);  // no rotation
    initPoseMsg.pose = initPose;

    // Publish the PoseStamped message for car2's initial pose
    pub.publish(initPoseMsg);
}

//ex3
void publishGoals(ros::Publisher& pub) {

    // Create a GoalPoseArray message for goals
    mushr_coordination::GoalPoseArray goalsMsg;      

      goalsMsg.scale = 2;
      goalsMsg.minx = 0;
      goalsMsg.miny = 0;
      goalsMsg.maxx = 4;
      goalsMsg.maxy = 6;  
    
    geometry_msgs::PoseArray temp_pickup;
    temp_pickup.poses.resize(2);

    temp_pickup.poses[0].position.x = 1.5;
    temp_pickup.poses[0].position.y = 3;
    temp_pickup.poses[0].orientation = eulerToQuaternion(0,0,1.5708);
    temp_pickup.poses[1].position.x = 4;
    temp_pickup.poses[1].position.y = 6;
    temp_pickup.poses[1].orientation = eulerToQuaternion(0,0,0);
    goalsMsg.goals.push_back(temp_pickup);

    temp_pickup.poses[0].position.x = 3.5;
    temp_pickup.poses[0].position.y = 3;
    temp_pickup.poses[0].orientation = eulerToQuaternion(0,0,1.5708);
    temp_pickup.poses[1].position.x = 0;
    temp_pickup.poses[1].position.y = 6;
    temp_pickup.poses[1].orientation = eulerToQuaternion(0,0,3.1415);
    goalsMsg.goals.push_back(temp_pickup);

    temp_pickup.poses[0].position.x = 2.5;
    temp_pickup.poses[0].position.y = 3.5;
    temp_pickup.poses[0].orientation = eulerToQuaternion(0,0,-1.5708);
    temp_pickup.poses[1].position.x = 3;
    temp_pickup.poses[1].position.y = 0;
    temp_pickup.poses[1].orientation = eulerToQuaternion(0,0,1.5708);
    goalsMsg.goals.push_back(temp_pickup);

    temp_pickup.poses[0].position.x = 0.5;
    temp_pickup.poses[0].position.y = 3.5;
    temp_pickup.poses[0].orientation = eulerToQuaternion(0,0,-1.5708);
    temp_pickup.poses[1].position.x = 0;
    temp_pickup.poses[1].position.y = 0;
    temp_pickup.poses[1].orientation = eulerToQuaternion(0,0,-1.5708);
    goalsMsg.goals.push_back(temp_pickup);

    // Publish the GoalPoseArray message for goals
    pub.publish(goalsMsg);

}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mushr_coordination_publisher");
    ros::NodeHandle nh;

    // Create publishers for the three topics
    ros::Publisher obstaclesPub = nh.advertise<geometry_msgs::PoseArray>("/mushr_coordination/obstacles", 10);

    ros::Publisher initPosePub1 = nh.advertise<geometry_msgs::PoseStamped>("/car1/init_pose", 10);
    ros::Publisher initPosePub2 = nh.advertise<geometry_msgs::PoseStamped>("/car2/init_pose", 10);
    ros::Publisher initPosePub3 = nh.advertise<geometry_msgs::PoseStamped>("/car3/init_pose", 10);
    ros::Publisher initPosePub4 = nh.advertise<geometry_msgs::PoseStamped>("/car4/init_pose", 10);

    ros::Publisher goalsPub = nh.advertise<mushr_coordination::GoalPoseArray>("/mushr_coordination/goals", 10);

    ros::spinOnce();
  
    
    ros::Rate rate(1);  // 1 Hz
    while (ros::ok()) {
        // Run the three publishers in separate functions
    publishObstacles(obstaclesPub);
    publishInitPose(initPosePub1,4, 0, 1.5708);
    publishInitPose(initPosePub2,2, 6, -1.5708);
    publishInitPose(initPosePub3,0, 0, 1.5708);
    publishInitPose(initPosePub4,0, 6, -1.5708);
    publishGoals(goalsPub);
    
        ros::spinOnce();
        rate.sleep();
    }
  

    return 0;
}
