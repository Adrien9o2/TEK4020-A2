#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>

ros::Publisher* pub_link_state;
gazebo_msgs::ModelState modelState;

void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  int index = 0;
  for (int i=0;i<msg->name.size();i++)
  {
    if ( std::string("camera").compare(msg->name[i]) == 0 )
    {
      index = i;
      break;
    }
  }
  
  modelState.pose = msg->pose[index];
}

void cameraTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  modelState.model_name = "camera";
  modelState.twist = *msg;
  
  pub_link_state->publish(modelState);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_controller");
  ros::NodeHandle nh;
  
  /* Allocate publishers */
  ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  pub_link_state = &pub;

  /* Allcate subscibers */
  ros::Subscriber sub_twist = nh.subscribe("camera_twist", 1000, cameraTwistCallback);
  ros::Subscriber sub_model_state = nh.subscribe("/gazebo/model_states", 1000, modelStateCallback);
  
  ros::spin();
  
  geometry_msgs::Twist t;
  modelState.twist = t;
  pub_link_state->publish(modelState);
  
}