#include "StereoSolver.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "stereo_solver");
  ros::NodeHandle nh;

  StereoSolver stereo_solver(nh);

  ros::Rate loop_rate(50); // 50 hz

  ros::spin();

  return 0;
}
