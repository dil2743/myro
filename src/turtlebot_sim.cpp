/*
*   to run the node first "run turtlesim_node"
*   this code will allign the robot at 180 (considring the wall is in that direction )
*   one the turtlebot is alligned then it will check and move the bot to a distance closer to x = 3.0 (simlar to  30cm distance from wall)
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "turtlesim/Pose.h"
#include "sstream"

using namespace std;
float current_orientation = 0.0;
float current_position = 0.0 ;
bool IS_POSE_AVAILABLE = false ,IS_ROBOT_ALLIGNED = false;
const float stopping_x_axis = 3.0 ; //safe distance
const float orentation_angle = 180 ; // orientation angle

void myPoseCallback(const turtlesim::Pose msg){
    // ROS_INFO("Robot Pose is :\n x:%f, y:%f ,theta:%f,linear:%f, angular:%f ",msg.x,msg.y,
    // msg.theta,msg.linear_velocity,msg.angular_velocity);
    current_orientation = msg.theta * 57.2958 ; //degree
    if(current_orientation < 0 )
        current_orientation = 360 + current_orientation;
    current_position = msg.x;
    IS_POSE_AVAILABLE = true;
}


int main(int argc ,char** argv){
    
    ros::init(argc,argv,"myTurtleSim");
    ros::NodeHandle myHandle;
    ros::Publisher myPublisher = myHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
    ros::Subscriber mySubscriber = myHandle.subscribe("/turtle1/pose",1,myPoseCallback);
    
    ros::Rate loop_rate(10);

    while (ros::ok){
        
        geometry_msgs::Twist cmd_var;
        if (IS_POSE_AVAILABLE == false ) {
            ROS_INFO("Waiting for turtlrbot to connect");
            ros::Duration(1).sleep();
            cmd_var.angular.z = 0;
        }else{
        /*
        *   Let us asume that the wall is at 180 i,e -X axis 
        *   
        */      
            if(current_orientation > orentation_angle+5 ){

            cmd_var.angular.z = 1;
            }else if(current_orientation < orentation_angle-5 ){
                cmd_var.angular.z = 1;
            }else if(IS_ROBOT_ALLIGNED == false){
                
                if(current_position < stopping_x_axis-0.1) cmd_var.linear.x = -1.0;
                else if(current_position > stopping_x_axis+0.1 ) cmd_var.linear.x = 1.0;
                else{
                    ROS_WARN("Robot Alligned");
                    IS_ROBOT_ALLIGNED = true ;
                    cmd_var.linear.x = 0 ;
                }
                cmd_var.angular.z = 0;
            }
        }
         myPublisher.publish(cmd_var);
        ros::spinOnce();
        loop_rate.sleep();
    }
}