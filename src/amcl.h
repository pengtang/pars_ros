#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#define MAX_BUF 1024

template <typename T>
std::string to_string(T value)
{
	std::ostringstream os ;
	os << value ;
	return os.str() ;
}

extern	geometry_msgs::PoseWithCovarianceStamped amcl_pose; // use the data in amcl.cpp
geometry_msgs::PoseWithCovarianceStamped amcl_pose_cur;
//geometry_msgs::PoseWithCovarianceStamped::Pose pose;
//geometry_msgs::PoseWithCovarianceStamped::float64[36] covariance;

class AMCL
{
public:
	std::string amcl_map; // absolute directory

	// constructor, initialize the amcl
	// CURRENTLY NOT WORKING, NEEDS TO BOOT MANUALLY
	AMCL(std::string amcl_map)
	{
		pid_t pid1 = fork();
		if (pid1 == 0)
			system("roscore");
		else if (pid1 > 0)
		{
			usleep(5000000);
			pid_t pid2 = fork();
			if (pid2 == 0)
			{
				std::string temp_str = "rosrun map_server map_server ";
				temp_str += amcl_map;
				system(temp_str.c_str());
			}
			else if (pid2 > 0)
			{
				usleep(5000000);
				pid_t pid3 = fork();
				if (pid3 == 0)
					system("roslaunch stdr_server stdr.launch");
				else if (pid3 > 0)
				{
					usleep(5000000);
					pid_t pid4 = fork();
					if (pid4 == 0)
						system("roslaunch stdr_server stdr_localization.launch");
					else if (pid4 > 0)
					{
						usleep(5000000);
						return;
					}
					else
						std::cerr<<"pid4 fork error"<<std::endl;
				}
				else
				{
					std::cerr<<"pid3 fork error"<<std::endl;
				}
			}
			else
			{
				std::cerr<<"pid2 fork error"<<std::endl;
			}
		}
		else
		{
			std::cerr<<"pid1 fork error"<<std::endl;
		}
		
	}

	AMCL(){}
	// double v = 0.2;
	// double delta_omega = 0.17;
	// double s = atof(argv[1]);
	// double spin_angle = atof(argv[2]);
	std::vector<double> AMCL_run(double s, double spin_angle)
	{
		std::vector<double> amcl_encode;

		std::string temp_str = "rosrun pars_ros amcl " + to_string(s) + ' ' +  to_string(spin_angle);

		int total_steps, status;
		double v = 0.05;
		double delta_omega = 0.17;
		double delta_t = 1;

		pid_t pid = fork();

		if (pid > 0)
		{
			// parent process, print
			// wait child process end first 
			//std::cout<<"v is "<< v << std::endl;
			total_steps = s==0 ?  abs(spin_angle/delta_omega) : s/v;
			//std::cout<< "Delta_t is  "<< delta_t << "  Total step is "<< total_steps<< std::endl;
			//std::cout<< "Total sleep time is "<< delta_t * total_steps<< std::endl;
			std::cerr<<"parent thread, s:"<< s << ", angle:" << spin_angle << std::endl;	
			std::cout<<"About to sleep "<<std::endl;
			usleep(int(1000000 * delta_t * total_steps) + 1000000);
			std::cout<<"Sleep finished"<<std::endl;
			//waitpid(pid, &status, 0);
			amcl_encode =  AMCL_print();
			return amcl_encode;
		}
		else if (pid == 0)
		{
			// child process, call execution function
			std::cerr<<"child thread, s:"<< s << ", angle:" << spin_angle << std::endl;
    		system(temp_str.c_str());
    		exit(0); // end the copy of fork on this side
		}
		else
			std::cerr<<"Fail to fork new process"<<std::endl;
	}
	std::vector<double> AMCL_print()
	{
	    int fd;
	    char * myfifo = "/tmp/myfifo";
	    //char buf[MAX_BUF];
	    double x, y, orientation_cos, orientation_sin;
	    double covariance[36];
	    std::vector<double> amcl_encapsulate;
	    /* open, read, and display the message from the FIFO */
	    fd = open(myfifo, O_RDONLY);
	    //std::cout<<"File open is called!"<<std::endl;
	    if (fd != -1)
	    {
		    //read(fd, buf, MAX_BUF);
		    read(fd, &x, sizeof(double));
		    read(fd, &y, sizeof(double));
		    read(fd, &orientation_cos, sizeof(double));
		    read(fd, &orientation_sin, sizeof(double));

		    for (int i=0; i<36; i++)
		    	read(fd, &covariance[i], sizeof(double));

		    close(fd);
		    std::cout<<"(x, y): (" << x <<", "<< y <<")"<<std::endl;
		    std::cerr<<"(x, y): (" << x <<", "<< y <<")"<<std::endl;
		    // std::cout<<x<<"  "<<y<<"  "<<orientation_cos<< "  " << orientation_sin <<std::endl;
		    // for (int i = 0;i<36;i++)
		    // 	std::cout<<covariance[i]<<"  ";
		    // printf("\n");
		    amcl_encapsulate.push_back(x);
		    amcl_encapsulate.push_back(y);
		    amcl_encapsulate.push_back(orientation_cos);
		    amcl_encapsulate.push_back(orientation_sin);
		    amcl_encapsulate.push_back(covariance[0]);//xx
		    amcl_encapsulate.push_back(covariance[1]);//xy
		    amcl_encapsulate.push_back(covariance[6]);//yx
		    amcl_encapsulate.push_back(covariance[7]);//yy
		    
		    std::cerr<<"variance for xy is "<< covariance[1]<< "  "<< covariance[6]<<std::endl;

		    return amcl_encapsulate;
		}
		else // If the pipe does not exist
		{
			//std::cout<<"Pipe does not exist"<<std::endl;
			std::cerr<<"Pipe does not exist, use the latest amcl data"<<std::endl;
			// Use the latest amcl data
			amcl_encapsulate.push_back(amcl_pose.pose.pose.position.x);
			amcl_encapsulate.push_back(amcl_pose.pose.pose.position.y);
			amcl_encapsulate.push_back(amcl_pose.pose.pose.orientation.w);
			amcl_encapsulate.push_back(amcl_pose.pose.pose.orientation.z);
			amcl_encapsulate.push_back(amcl_pose.pose.covariance[0]);
			amcl_encapsulate.push_back(amcl_pose.pose.covariance[1]);
			amcl_encapsulate.push_back(amcl_pose.pose.covariance[6]);
			amcl_encapsulate.push_back(amcl_pose.pose.covariance[7]);		
			return amcl_encapsulate;	
		}

	}

};
