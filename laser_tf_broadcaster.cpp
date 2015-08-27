#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <tf/transform_listener.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "laser_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(50);
	
	tf::TransformBroadcaster broadcaster;

	while(n.ok()){

		/********************** for stdr *************************/
		
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try{
			listener.waitForTransform("/map_static", "/robot0", ros::Time(0), ros::Duration(50.0));
			listener.lookupTransform("/map_static", "/robot0", ros::Time(0), transform);

			//printf("robot_map_origin = (%0.3f, %0.3f, %0.3f)\n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
			
			//robot_map_pose.pose.pose.position.x = msg.pose.pose.position.x + transform.getOrigin().x();
			//robot_map_pose.pose.pose.position.y = msg.pose.pose.position.y + transform.getOrigin().y();
			//printf("robot_map_position = (%0.3f, %0.3f)\n", robot_map_position_x, robot_map_position_y);
			//printf("robot_position = (%0.3f, %0.3f)\n", msg.pose.pose.position.x, msg.pose.pose.position.y);
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
		}

		double x = transform.getOrigin().x();
		double y = transform.getOrigin().y();
		double z = transform.getRotation().z();
		double w = transform.getRotation().w();

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
				ros::Time::now(),"base_link", "robot0_laser_0"));

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,z,w), tf::Vector3(x,y,0)),
				ros::Time::now(),"odom", "base_link"));		

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-5,-5,0)),
				ros::Time::now(),"map", "odom"));
		
		/***************************** end stdr ********************************/

		r.sleep();
	}
}
