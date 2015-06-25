#include "amcl.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
using namespace std;

geometry_msgs::PoseWithCovarianceStamped amcl_pose;
int main()
{
	//AMCL a("/home/pengtang/catkin_ws/src/stdr_simulator/stdr_resources/maps/map.yaml");
	AMCL a;

	vector<double> amcl_decipher;

	// double v = 0.2;
	// double s = atof(argv[1]);
	// double delta_omega = 0.17;
	// double spin_angle = atof(argv[2]);

	amcl_decipher = a.AMCL_run(0.1, 0);
	cout<<amcl_decipher[0]<<"  "<<amcl_decipher[1]<<endl;

	amcl_decipher = a.AMCL_run(0.2, 0);
	cout<<amcl_decipher[0]<<"  "<<amcl_decipher[1]<<endl;

	amcl_decipher = a.AMCL_run(0.1, 0);
	cout<<amcl_decipher[0]<<"  "<<amcl_decipher[1]<<endl;

	return 0;
}
