#include "amcl.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::PoseWithCovarianceStamped amcl_pose;
int main()
{
//	AMCL a("/home/pengtang/catkin_ws/src/stdr_simulator/stdr_resources/maps/map.yaml");
	AMCL a;
//	a.AMCL_run(1, 1, 0.3, 1.5, 0, 0);
//	a.AMCL_run(1, 1, 0.3, 0.9, 0, 0);
//	a.AMCL_run(1, 1, 0.5, 2.0, 0, 0);

	a.AMCL_run(1, 1, 0.4, 0.8, 0, 0);
	a.AMCL_run(1, 1, 0.3, 0.6, 0, 0);
	a.AMCL_run(1, 1, 1, 2, 0, 0);


	//a.AMCL_print();
	
	//new AMCL("/home/pengtang/catkin_ws/src/stdr_simulator/stdr_resources/maps/map.yaml");
	//robot_localize.AMCL_run();
	//robot_localize.AMCL_print();
	return 0;
}


//GET: current time step t, delta_t(constant say 0.1 sec), v, delta_omega -> customized_teleop
//NOTICE: one of [delta_t, v] [delta_o] has to be 0

//system(rosrun pars_ros customized_teleop delta_t, v, delta_o);

//system(rostopic echo amcl_pose) and redirect to a file

//parse the file, extract the two variables pose and variance.
