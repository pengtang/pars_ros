#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "amcl.h"

// Input argument(first two are not supported for user to modify through input argument, with default value here)
//                         : <delta_t>, <v>, s(travel_distance), spin_angle

//int amcl_msg_count = 0;
int tf_msg_count = 0;
double distance_traveled = 0;
double angle_spinned = 99999999;
double EPISILON_DISTANCE = 0.05;
double EPISILON_ANGLE = 0.03;
double travel_distance;

// amcl_pose is declared as a extern variable in "amcl.h"
geometry_msgs::PoseWithCovarianceStamped amcl_pose, amcl_initial;

// file for debugging
std::ofstream file;

void speedMessageReceived(const geometry_msgs::Twist &msg)
{
    //file<<"Current time step is "<<current_time_step<<std::endl
    // <<"Current linear speed is "<< msg.linear.x<<std::endl
    // <<"Current angular speed is "<<msg.angular.z<<std::endl;
    //ROS_INFO_STREAM("The speed is "<< );
}

void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    amcl_pose = msg;
    // if (amcl_msg_count == 0)
    // {
    //   amcl_initial = msg;
    //   amcl_msg_count++;
    // }
    // else
    // {
    //   // Angle spinned calculation is incorrect...
    //   //angle_spinned = std::abs(msg.pose.pose.orientation.z - amcl_initial.pose.pose.orientation.z);
      
    //   //std::cerr << "mytest: z " << asin(msg.pose.pose.orientation.z) << " w " << acos(msg.pose.pose.orientation.w) << std::endl;
    //   double my_temp1 = atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2;
    //   double my_temp2 = atan2(amcl_initial.pose.pose.orientation.z, amcl_initial.pose.pose.orientation.w) * 2;

    //   angle_spinned = std::abs(my_temp1 - my_temp2);
      
    //   std::cout<<"my_temp1 is  "<< my_temp1<<"  my_temp2 is  "<<my_temp2<<" angle_spinned is "<< angle_spinned<<std::endl;

    //   distance_traveled = sqrt( pow((msg.pose.pose.position.x - amcl_initial.pose.pose.position.x),2) +
    //     pow((msg.pose.pose.position.y - amcl_initial.pose.pose.position.y),2) );
    //     std::cerr<<"distance_traveled is "<<distance_traveled<<std::endl;       
    // }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_three_second");
  ros::NodeHandle nh;
  
  ros::Subscriber speed = nh.subscribe("robot0/cmd_vel", 1, &speedMessageReceived) ;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
  ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amclMessageReceived);

  geometry_msgs::Twist msg;

//  double delta_t = atof(argv[1]); //DOES NOT APPLY FOR NOW
//  double v = atof(argv[2]); // DOES NOT APPLY FOR NOW, PLEASE MAKE THIS RELATIVELY SMALL COMPARED TO s

  // Set parameters
  double v = 0.02;
  double s = atof(argv[1]);
  
  travel_distance = s;
 // double delta_omega = atof(argv[3]);
  double delta_omega = 0.17;
  double spin_angle = atof(argv[2]);
  
  	std::cout<<"s/travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
  	std::cerr<<"s/travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
  
  //if (v == 0 && spin_angle == 0)
  if (s == 0 && spin_angle == 0)
    ROS_DEBUG_STREAM("Input is incorrect, it's not moving forward or spinning");
  //if (v != 0 && spin_angle != 0)
  if (s != 0 && spin_angle != 0)
    ROS_DEBUG_STREAM("Moving forward and spinning at the same time, currently not supported");
  //int total_steps = v==0 ?  abs(spin_angle/delta_omega) : s/v;
  int total_steps = s==0 ?  abs(spin_angle/delta_omega) : s/v;

  //std::cout<<spin_angle<<"  "<<delta_omega<<"  "<<abs(spin_angle/delta_omega)<<std::endl;
  //std::cout<<"v is "<<v<<"s/v = "<<s/v<< "   abs(spin_angle/delta_omega) = "<< std::abs(spin_angle/delta_omega)<<std::endl;
  //std::cout<<"Total steps is "<<total_steps<<std::endl;
  //std::cout<<StartTimeStep<<"  "<<delta_t<<"  "<<v<<"  "<<s<<"  "<<delta_omega<<std::endl;
  double BASE_LINEAR_SPEED = v, BASE_ANGULAR_SPEED = delta_omega;
  int count = 0;

//  file.open("speed_file.txt"); // for debug purpose

  bool movement_forward = s==0 ? false : true;
  bool movement_spin = not movement_forward;

// ???
  double CLOCK_SPEED = 0.05;
  ros::Rate rate(1/CLOCK_SPEED);
  tf::TransformListener listener(ros::Duration(1/CLOCK_SPEED));

  geometry_msgs::PointStamped offset_point;
  geometry_msgs::PointStamped tf_initial;
  geometry_msgs::PointStamped base_point;

  // If the angle need to spin is negative, spin clockwise
  if (spin_angle < 0)
      BASE_ANGULAR_SPEED *= -1;

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
    while(ros::ok() && std::abs(distance_traveled - s)>EPISILON_DISTANCE )
    {
        msg.linear.x = BASE_LINEAR_SPEED;
        pub.publish(msg);
        // ROS_INFO_STREAM("The robot is now moving forward!");
        std::cerr<<"distance_traveled is "<< distance_traveled << " s is "<< s <<"   "<<std::endl;
        std::cerr<<"distance to goal abs(distance_traveled - s) is "<< std::abs(distance_traveled - s) << std::endl;
        //count++;

        offset_point.header.frame_id = "odom";

        offset_point.point.x = 17.0;
        offset_point.point.y = 17.0;
        offset_point.point.z = 0.0;

        try{
          listener.transformPoint("map", offset_point, base_point);

          if (tf_msg_count != 0)
            std::cerr<<"initial pos: "<<tf_initial.point.x<<"  "<< tf_initial.point.y<<std::endl;
          std::cerr<<"current pos: "<<base_point.point.x<<"  "<< base_point.point.y<<std::endl;

          if (tf_msg_count == 0 && base_point.point.x!=0)
          {
              tf_initial.point.x = base_point.point.x;
              tf_initial.point.y = base_point.point.y;
              tf_msg_count++;
          }
          distance_traveled = sqrt( pow((base_point.point.x - tf_initial.point.x),2) + pow((base_point.point.y - tf_initial.point.y),2) );
        }
        catch(tf::TransformException& ex){
          if (tf_msg_count == 0 && base_point.point.x!=0)
          {
              tf_initial.point.x = base_point.point.x;
              tf_initial.point.y = base_point.point.y;
              tf_msg_count++;
          }
          ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"map\": %s", ex.what());
        }   
             
        ros::spinOnce();
        rate.sleep();
    }

  // file.close(); //for debug purpose

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

  // Write the answer to pipe
  int fd;
  char * myfifo = "/tmp/myfifo";

  /* create the FIFO (named pipe) */
  //mkfifo(myfifo, 0666);

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
  
  //ros::shutdown();

  /* remove the FIFO */
  //unlink(myfifo);
//  std::cout<<amcl_pose<<std::endl;

}
