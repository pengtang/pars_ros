

//GET: current time step t, delta_t(constant say 0.1 sec), v, delta_omega -> customized_teleop
//NOTICE: one of [delta_t, v] [delta_o] has to be 0

//system(rosrun pars_ros customized_teleop delta_t, v, delta_o);

//system(rostopic echo amcl_pose) and redirect to a file

//parse the file, extract the two variables pose and variance.
