#include <ros/ros.h>
#include "vdm_ins.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "vdm_ins");
  ros::NodeHandle nh("~");
  //getchar();

  StateEstimator estimator;
  estimator.onInit(nh);
  estimator.sub_rpm_  = nh.subscribe("rpm",  10,&StateEstimator::rpm_callback,  &estimator);
  estimator.sub_odom_ = nh.subscribe("pose", 10,&StateEstimator::pose_callback, &estimator);
  estimator.sub_imu_  = nh.subscribe("imu",  10,&StateEstimator::imu_callback,  &estimator);

  estimator.pub_estimates_ = nh.advertise<quadrotor_vdm_aided_ins::ParameterEstimates>("param_estimates", 10);

  ros::spin();

  return 0;
}