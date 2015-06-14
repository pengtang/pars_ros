#include "amcl.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::PoseWithCovarianceStamped amcl_pose;
int main()
{
//	AMCL a("/home/pengtang/catkin_ws/src/stdr_simulator/stdr_resources/maps/map.yaml");
	AMCL a;

	// double v = 0.2;
	// double s = atof(argv[1]);
	// double delta_omega = 0.17;
	// double spin_angle = atof(argv[2]);

	a.AMCL_run(0.2, 0);

	a.AMCL_run(0.4, 0);

	a.AMCL_run(0, 3.14);

	return 0;
}
