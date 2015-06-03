#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <iostream>
#include <sstream>

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
	AMCL(std::string amcl_map)
	{
		system("roscore");
		std::string temp_str = "rosrun map_server map_server ";
		temp_str += amcl_map;
		system(temp_str.c_str());
		system("roslaunch stdr_server stdr.launch");
		system("roslaunch stdr_server stdr_localization.launch");
	}

	AMCL(){}
	// Return void or PoseWithCovarianceStamped? and HOW?
	// int StartTimeStep = atoi(argv[1]);
	// double delta_t = atof(argv[2]);
	// double v = atof(argv[3]);
	// double s = atof(argv[4]);
	// double delta_omega = atof(argv[5]);
	// double spin_angle = atof(argv[6]);
	void AMCL_run(int StartTimeStep, double delta_t, double v, double s, double delta_omega, double spin_angle)
	{
		std::string temp_str = "rosrun pars_ros amcl " + to_string(StartTimeStep) + ' '
		 + to_string(delta_t) + ' ' + to_string(v) + ' ' + to_string(s) + ' '
		 + to_string(delta_omega) + ' ' + to_string(spin_angle);

		int total_steps, status;
		pid_t pid = fork();

		if (pid > 0)
		{
			// parent process, print
			// wait child process end first 
			//std::cout<<"v is "<< v << std::endl;
			total_steps = v==0 ?  abs(spin_angle/delta_omega) : s/v;
			//std::cout<< "Delta_t is  "<< delta_t << "  Total step is "<< total_steps<< std::endl;
			//std::cout<< "Total sleep time is "<< delta_t * total_steps<< std::endl;		
			std::cout<<"About to sleep "<<std::endl;
			usleep(int(1000000 * delta_t * total_steps) + 1000000);
			std::cout<<"Sleep finished"<<std::endl;
			//waitpid(pid, &status, 0);
			AMCL_print();
		}
		else if (pid == 0)
		{
			// child process, call execution function
			
    		system(temp_str.c_str());
    		exit(0); // end the copy of fork on this side
		}
		else
			std::cerr<<"Fail to fork new process"<<std::endl;
	}
	void AMCL_print()
	{
	    int fd;
	    char * myfifo = "/tmp/myfifo";
	    //char buf[MAX_BUF];
	    double x, y, orientation;
	    double covariance[36];
	    /* open, read, and display the message from the FIFO */
	    fd = open(myfifo, O_RDONLY);
	    //std::cout<<"File open is called!"<<std::endl;
	    if (fd != -1)
	    {
		    //read(fd, buf, MAX_BUF);
		    read(fd, &x, sizeof(double));
		    read(fd, &y, sizeof(double));
		    read(fd, &orientation, sizeof(double));

		    for (int i=0; i<36; i++)
		    	read(fd, &covariance[i], sizeof(double));

		    std::cout<<x<<"  "<<y<<"  "<<orientation<<std::endl;
		    for (int i = 0;i<36;i++)
		    	std::cout<<covariance[i]<<"  ";
		    printf("\n");
		    close(fd);
		}
		else // If the pipe does not exist
		{
			//std::cout<<"Pipe does not exist"<<std::endl;
			std::cerr<<"Pipe does not exist"<<std::endl;
		}

	}

};