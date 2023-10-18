#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
    
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    

int main(int argc, char** argv){
      ros::init(argc, argv, "simple_navigation_goals");
    
     //tell the action client that we want to spin a thread by default
MoveBaseClient ac("move_base", true);
  
     //wait for the action server to come up
     while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up"); 
     }
   
     std::vector <move_base_msgs::MoveBaseGoal> goals;
     for (int i=1;i<5;i++){
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.y = i;
      goal.target_pose.pose.position.x = 5;
      goal.target_pose.pose.orientation.w = 1;
      goals.push_back(goal);
     }

     //we'll send a goal to the robot to move 1 meter forward
     for (int i=0 ; i<goals.size(); i++){
     ROS_INFO("Sending goal");
     ac.sendGoal(goals.at(i));
   
     ac.waitForResult();
   
     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       ROS_INFO("Hooray, the base reached goal [%i] ",i);
     else
       ROS_INFO("The base failed to move forward 1 meter for some reason");
     }
     return 0;
   }