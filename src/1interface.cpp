#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// argument: StartTimeStep, delta_t, v, s, delta_omega
// For simplicity, right now the dropoff speed is not considered, namely the speed is locked as a constant
int current_time_step;
std::ofstream file;
void speedMessageReceived(const geometry_msgs::Twist &msg)
{
    file<<"Current time step is "<<current_time_step<<std::endl
    <<"Current linear speed is "<< msg.linear.x<<std::endl
    <<"Current angular speed is "<<msg.angular.z<<std::endl;
    //ROS_INFO_STREAM("The speed is "<< );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_three_second");
  ros::NodeHandle nh;
  
  ros::Subscriber speed = nh.subscribe("RosAria/cmd_vel", 1, &speedMessageReceived) ;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  geometry_msgs::Twist msg;

  int StartTimeStep = atoi(argv[1]);
  double delta_t = atof(argv[2]);
  double v = atof(argv[3]);
  double s = atof(argv[4]);
  double delta_omega = atof(argv[5]);
  double spin_angle = atof(argv[6]);

  if (v == 0 && spin_angle == 0)
    ROS_DEBUG_STREAM("Input is incorrect, it's not moving forward or spinning");
  if (v != 0 && spin_angle != 0)
    ROS_DEBUG_STREAM("Moving forward and spinning at the same time, currently not supported");
  int total_steps = v==0 ?  abs(spin_angle/delta_omega) : s/v;

  //std::cout<<spin_angle<<"  "<<delta_omega<<"  "<<abs(spin_angle/delta_omega)<<std::endl;
  std::cout<<"s/v = "<<s/v<< "   abs(spin_angle/delta_omega) = "<< abs(spin_angle/delta_omega)<<std::endl;
  std::cout<<"Total steps is "<<total_steps<<std::endl;
  //std::cout<<StartTimeStep<<"  "<<delta_t<<"  "<<v<<"  "<<s<<"  "<<delta_omega<<std::endl;
  double BASE_LINEAR_SPEED = v, BASE_ANGULAR_SPEED = delta_omega, CLOCK_SPEED = delta_t;
  int count = 0;
  ros::Rate rate(1/CLOCK_SPEED);

  file.open("speed_file.txt");
  while(ros::ok() && count<total_steps)
  {
      current_time_step = StartTimeStep + count;
      msg.linear.x = BASE_LINEAR_SPEED;
      msg.angular.z = BASE_ANGULAR_SPEED;
      pub.publish(msg);
      ROS_INFO_STREAM("The robot is now moving forward!");
      //system("rostopic ecsho...");
      count++;
      ros::spinOnce();
      rate.sleep();
  }
  file.close();

  // make the robot stop
  
  for (int i = 0; i < 2; i++)
  {  
      msg.linear.x = 0;
      msg.linear.y = 0;
      msg.linear.z = 0;
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = 0;
      pub.publish(msg);
  }
    ROS_INFO_STREAM("The robot finished moving forward three seconds!");
    
    // Guard, make sure the robot stops.
  rate.sleep();
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  pub.publish(msg); 

}