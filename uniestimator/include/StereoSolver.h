#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class StereoSolver {

// Constructor & Destructor
public:
  StereoSolver(ros::NodeHandle& nh){

    batchImgPub_ = nh.advertise<sensor_msgs::Image>("/stereo_batch/image_raw", 2);

    leftImgSub_ = nh.subscribe("/camera/left/image_raw", 1, &StereoSolver::leftImgCallback, this);
    rightImgSub_ = nh.subscribe("/camera/right/image_raw", 1, &StereoSolver::rightImgCallback, this);

    img_left_flag_ = false;
    img_right_flag_ = false;
    
  }
  
  ~StereoSolver(){};

// Public member functions
public:



private:

  void leftImgCallback( const sensor_msgs::ImageConstPtr& msg) { img_left_ = *msg; img_left_flag_ = true; stitchImg(); }
  void rightImgCallback( const sensor_msgs::ImageConstPtr& msg) { img_right_ = *msg; img_right_flag_ = true; stitchImg(); }

  bool featureMatching();
  void stitchImg();
  void publishBatchImg();

private:


  ros::Publisher batchImgPub_;
  ros::Subscriber leftImgSub_;
  ros::Subscriber rightImgSub_;

  sensor_msgs::Image img_left_;
  sensor_msgs::Image img_right_;

  bool img_left_flag_;
  bool img_right_flag_;

};
