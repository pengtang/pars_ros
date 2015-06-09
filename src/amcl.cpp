#include <iostream>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "amcl.h"


// argument: StartTimeStep, delta_t, v, s, delta_omega
// For simplicity, right now the dropoff speed is not considered, namely the speed is locked as a constant
int current_time_step;
int amcl_msg_count = 0;
double distance_traveled = 99999999;
double angle_spinned = 99999999;
double EPISILON_DISTANCE = 0.1;
double EPISILON_ANGLE = 0.1;
double travel_distance;

// amcl_pose is declared as a extern variable in "amcl.h"
geometry_msgs::PoseWithCovarianceStamped amcl_pose, amcl_initial;
std::ofstream file;

void speedMessageReceived(const geometry_msgs::Twist &msg)
{
    // file<<"Current time step is "<<current_time_step<<std::endl
    // <<"Current linear speed is "<< msg.linear.x<<std::endl
    // <<"Current angular speed is "<<msg.angular.z<<std::endl;
    //ROS_INFO_STREAM("The speed is "<< );
}

void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    amcl_pose = msg;
    if (amcl_msg_count == 0)
    {
      amcl_initial = msg;
      amcl_msg_count++;
    }
    else
    {
      angle_spinned = std::abs(msg.pose.pose.orientation.z - amcl_initial.pose.pose.orientation.z);

      distance_traveled = sqrt( pow((msg.pose.pose.position.x - amcl_initial.pose.pose.position.x),2) +
        pow((msg.pose.pose.position.y - amcl_initial.pose.pose.position.y),2) );
    }

    // file<<"amcl_msg_count is "<<amcl_msg_count<<std::endl<<"amcl_initial is "<<amcl_initial<<std::endl<<
    // "msg is "<<msg<<std::endl<<"distance_traveled is "<<distance_traveled<<std::endl<<
    // "distane remaining/past is "<<abs(distance_traveled - travel_distance)<<std::endl;

    // file<<"current orientation is "<<msg.pose.pose.orientation.z<< std::endl<<
    // "current orientation (x) is "<<msg.pose.pose.orientation.x<<std::endl<<
    // "current orientation (y) is "<<msg.pose.pose.orientation.y<<std::endl<<
    // "current orientation (w) is "<<msg.pose.pose.orientation.w<<std::endl<<
    // "initial orientation is " << amcl_initial.pose.pose.orientation.z<< std::endl<< std::endl;

    // file<<"distance_traveled is "<<distance_traveled<<std::endl<<
    // "distance needs to travel is "<<travel_distance<<std::endl<<
    // "distane remaining/past is "<<distance_traveled - travel_distance<<std::endl<<
    // "distane remaining/past is(abs) "<<std::abs(distance_traveled - travel_distance)<<std::endl<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_three_second");
  ros::NodeHandle nh;
  
  ros::Subscriber speed = nh.subscribe("robot0/cmd_vel", 1, &speedMessageReceived) ;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
  ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amclMessageReceived);

  geometry_msgs::Twist msg;

  int StartTimeStep = atoi(argv[1]);
  double delta_t = atof(argv[2]);
  double v = atof(argv[3]);
  double s = atof(argv[4]);
  travel_distance = s;
  double delta_omega = atof(argv[5]);
  double spin_angle = atof(argv[6]);

  if (v == 0 && spin_angle == 0)
    ROS_DEBUG_STREAM("Input is incorrect, it's not moving forward or spinning");
  if (v != 0 && spin_angle != 0)
    ROS_DEBUG_STREAM("Moving forward and spinning at the same time, currently not supported");
  int total_steps = v==0 ?  abs(spin_angle/delta_omega) : s/v;

  //std::cout<<spin_angle<<"  "<<delta_omega<<"  "<<abs(spin_angle/delta_omega)<<std::endl;
  //std::cout<<"v is "<<v<<"s/v = "<<s/v<< "   abs(spin_angle/delta_omega) = "<< std::abs(spin_angle/delta_omega)<<std::endl;
  //std::cout<<"Total steps is "<<total_steps<<std::endl;
  //std::cout<<StartTimeStep<<"  "<<delta_t<<"  "<<v<<"  "<<s<<"  "<<delta_omega<<std::endl;
  double BASE_LINEAR_SPEED = v, BASE_ANGULAR_SPEED = delta_omega;
  int count = 0;
  double CLOCK_SPEED = 0.2;
  ros::Rate rate(1/CLOCK_SPEED);

  file.open("speed_file.txt");

  bool movement_forward = v==0 ? false : true;
  bool movement_spin = not movement_forward;

  if (movement_spin)
    while(ros::ok() && count<int(spin_angle/BASE_ANGULAR_SPEED/CLOCK_SPEED) + 1)
    {
        msg.angular.z = BASE_ANGULAR_SPEED;
        pub.publish(msg);
        ROS_INFO_STREAM("The robot is now spinning!");
        std::cout<<"angle_spinned is "<<angle_spinned<<"    angle needs to spin is "<<spin_angle<<std::endl;
        //system("rostopic ecsho...");
        count++;
        ros::spinOnce();
        rate.sleep();
    }    
  else if (movement_forward)
    while(ros::ok() && std::abs(distance_traveled - s) > EPISILON_DISTANCE )
    {
        current_time_step = StartTimeStep + count;
        msg.linear.x = BASE_LINEAR_SPEED;
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
    ROS_INFO_STREAM("The robot finished moving forward/ spinning!");
  
    // Guard, make sure the robot stops.
  rate.sleep();
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  pub.publish(msg); 

  int fd;
  char * myfifo = "/tmp/myfifo";

  /* create the FIFO (named pipe) */
  mkfifo(myfifo, 0666);

  /* write "Hi" to the FIFO */
  fd = open(myfifo, O_WRONLY);
  ROS_INFO_STREAM("The pipe has been created!");
  //std::cout<<"The pipe has been created"<<std::endl;
  //ROS_INFO_STREAM("The value written is "<<amcl_pose.pose.pose.position.x);
  write(fd, (char *) &amcl_pose.pose.pose.position.x, sizeof(double));
  write(fd, (char *) &amcl_pose.pose.pose.position.y, sizeof(double));

  write(fd, (char *) &amcl_pose.pose.pose.orientation.w, sizeof(double));
  write(fd, (char *) &amcl_pose.pose.pose.orientation.z, sizeof(double));

  for (int i=0; i<36; i++)
    write(fd, (char *) &amcl_pose.pose.covariance[i], sizeof(double));

  close(fd);

  /* remove the FIFO */
  unlink(myfifo);
//  std::cout<<amcl_pose<<std::endl;

}
