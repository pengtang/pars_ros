#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "/home/pengtang/pars/amcl.h"

// Input argument(first two are not supported for user to modify through input argument, with default value here)
//                         : <delta_t>, <v>, s(travel_distance), spin_angle

//int amcl_msg_count = 0;
int tf_msg_count = 0;
double distance_traveled = 0;
double angle_spinned = 99999999;
double EPISILON_DISTANCE = 0.004;
double EPISILON_ANGLE = 0.03;
double travel_distance;

geometry_msgs::PoseWithCovarianceStamped amcl_pose, amcl_initial;

void speedMessageReceived(const geometry_msgs::Twist &msg) {}
void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    amcl_pose = msg;
    std::cerr << "AMCL pose: " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "go_three_second");
    ros::NodeHandle nh;
  
    ros::Subscriber speed = nh.subscribe("robot0/cmd_vel", 1, &speedMessageReceived) ;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
    ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amclMessageReceived);

    geometry_msgs::Twist msg;

    double v = 0.02;
    double s = atof(argv[1]);
    travel_distance = s;
    double delta_omega = 0.29;
    double spin_angle = atof(argv[2]);
  
    std::cout<<"travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
    std::cerr<<"travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
  
    if (s == 0 && spin_angle == 0) {
      ROS_DEBUG_STREAM("Input is incorrect, it's not moving forward or spinning");
    }
    if (s != 0 && spin_angle != 0) {
      ROS_DEBUG_STREAM("Moving forward and spinning at the same time, currently not supported");
    }
    
    int total_steps = s==0 ?  abs(spin_angle/delta_omega) : s/v;

    double BASE_LINEAR_SPEED = v, BASE_ANGULAR_SPEED = delta_omega;
    int count = 0;

    bool movement_forward = s==0 ? false : true;
    bool movement_spin = !movement_forward;

    tf::TransformListener listener;
    double CLOCK_SPEED = 0.1;
    ros::Rate rate(1/CLOCK_SPEED);
  
    //geometry_msgs::PointStamped offset_point;
    //offset_point.header.frame_id = "odom";
    //geometry_msgs::PointStamped tf_initial;
    //geometry_msgs::PointStamped base_point;
  
    tf::StampedTransform init_transform;
    tf::StampedTransform curr_transform;
    geometry_msgs::Pose stdrbot_pose;

    if (movement_spin) {
      while (nh.ok() && movement_spin && count <= int( std::abs(spin_angle)/BASE_ANGULAR_SPEED/CLOCK_SPEED)) {
          tf::StampedTransform transform;
        try {
          listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0));
          listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
        } catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        curr_transform = transform;
        //std::cerr<< "current pos: " << curr_transform.getOrigin().x() << "  " << curr_transform.getOrigin().y() << std::endl;
            
        if (spin_angle < 0) 
            msg.angular.z = -1 * BASE_ANGULAR_SPEED;
        else 
            msg.angular.z = BASE_ANGULAR_SPEED;

          pub.publish(msg);
        ROS_INFO_STREAM("The robot is now spinning!");
        std::cout << "angle_spinned is " << angle_spinned << " angle needs to spin is "<< spin_angle << std::endl;
        count++;
        ros::spinOnce();
        rate.sleep();     
      }
      std::cerr<< "current pos: " << curr_transform.getOrigin().x() << "  " << curr_transform.getOrigin().y() << std::endl;
              
    } 
    else if (movement_forward) {
      int move_count = 0;
        while (nh.ok() && std::abs(s - distance_traveled) > EPISILON_DISTANCE) {
          tf::StampedTransform transform;
          try {
            listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
          } 
          catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
          }

          if (move_count == 0) 
            init_transform = transform;
          
          curr_transform = transform;
          distance_traveled = sqrt(pow(transform.getOrigin().x() - init_transform.getOrigin().x(), 2) +
                                      pow(transform.getOrigin().y() - init_transform.getOrigin().y(), 2));
          msg.linear.x = BASE_LINEAR_SPEED;
          pub.publish(msg);
          std::cerr<< "initial pos: " << init_transform.getOrigin().x() << "  " << init_transform.getOrigin().y() << std::endl;
          std::cerr<< "current pos: " << curr_transform.getOrigin().x() << "  " << curr_transform.getOrigin().y() << std::endl;
          std::cerr<< "distance left to go is " << std::abs(distance_traveled - s) << std::endl;  
          //std::cerr<< "initial pos: " << init_transform.getOrigin().x() << "  " << init_transform.getOrigin().y() << std::endl;
          //std::cerr<< "current pos: " << curr_transform.getOrigin().x() << "  " << curr_transform.getOrigin().y() << std::endl;
          //std::cerr<< "distance left to go is " << std::abs(s - distance_traveled) << std::endl;      
          move_count++;
          ros::spinOnce();
          rate.sleep();
        }
  
    }

     // make the robot stop  
    for (int i = 0; i < 2; i++) {
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
 
    /* write "Hi" to the FIFO */
    fd = open(myfifo, O_WRONLY);
    ROS_INFO_STREAM("The pipe has been created!");
    stdrbot_pose.position.x = curr_transform.getOrigin().x()-5.0;
    stdrbot_pose.position.y = curr_transform.getOrigin().y()-5.0;

    stdrbot_pose.orientation.w = curr_transform.getRotation().w();
    stdrbot_pose.orientation.z = curr_transform.getRotation().z();
//    stdrbot_pose.orientation.w = tf::getYaw(curr_transform.getRotation());
    std::cerr<< "write in to PIPE: " << stdrbot_pose.position.x << "  " << stdrbot_pose.position.y << "  " 
    << stdrbot_pose.orientation.w <<std::endl;
   
    double UNCERTAINTY = 500;
    write(fd, (char *) &stdrbot_pose.position.x, sizeof(double));
    write(fd, (char *) &stdrbot_pose.position.y, sizeof(double));
    write(fd, (char *) &stdrbot_pose.orientation.w, sizeof(double));
    write(fd, (char *) &stdrbot_pose.orientation.z, sizeof(double));
//    write(fd, (char *) &amcl_pose.pose.pose.position.x, sizeof(double));
//    write(fd, (char *) &amcl_pose.pose.pose.position.y, sizeof(double));
//    write(fd, (char *) &amcl_pose.pose.pose.orientation.w, sizeof(double));
//    write(fd, (char *) &amcl_pose.pose.pose.orientation.z, sizeof(double));

    for (int i=0; i<36; i++)
      write(fd, (char *) &amcl_pose.pose.covariance[i], sizeof(double));

    close(fd);
  return 0;
    /* remove the FIFO */
    //unlink(myfifo);
}