#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace std;
int main()
{
  double previous_angle, INITIAL_ANGLE = 0;

  int angle_pipeopen, angle_pipewrite;
  char * anglefifo = "/tmp/anglefifo";

  angle_pipeopen = open(anglefifo, O_RDONLY);
  angle_pipewrite = open(anglefifo, O_WRONLY);

  if (angle_pipeopen != -1)
  {
    read(angle_pipeopen, &previous_angle, sizeof(double));
    cout<<"pipe exists, the value of previous_angle is "<<previous_angle<<endl;
  }
  else // If the pipe does not exist, build the pipe and initialize angle with 0.
  {
    cout<<"pipe does not exist, write to pipe"<<endl;
    mkfifo(anglefifo, 0666);
    cout<<"initial value is: "<< INITIAL_ANGLE <<endl; 
      write(angle_pipewrite, &INITIAL_ANGLE, sizeof(double));
      previous_angle = INITIAL_ANGLE;
    cout<<"pipe created, content has been written to the pipe"
  }

  return 0;
}
