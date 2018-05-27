#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

class StereoSolver {

// Constructor & Destructor
public:
  StereoSolver(ros::NodeHandle& nh){

    batchImgPub_ = nh.advertise<sensor_msgs::Image>("/stereo_batch/image_raw", 100);

    leftImgSub_ = nh.subscribe("/camera/left/image_raw", 1, &StereoSolver::leftImgCallback, this);
    rightImgSub_ = nh.subscribe("/camera/right/image_raw", 1, &StereoSolver::rightImgCallback, this);

    img_left_flag_ = false;
    img_right_flag_ = false;

    img_width_ = 752;
    img_height_ = 480;

  }
  
  ~StereoSolver(){};

// Public member functions
public:



private:

  sensor_msgs::Image cvMatToROSMsg(const cv::Mat image, const std::string frame_id = "", const std::string encoding = sensor_msgs::image_encodings::MONO8);
  cv::Mat ROSMsgToCvMat(const sensor_msgs::Image);

  void leftImgCallback(const sensor_msgs::ImageConstPtr&);
  void rightImgCallback(const sensor_msgs::ImageConstPtr&);
  void boundingBoxCallback(const darknet_ros_msgs::BoundingBoxes);

  bool featureMatching();
  void stitchImg();
  void publishBatchImg();



private:


  ros::Publisher batchImgPub_;
  ros::Subscriber leftImgSub_;
  ros::Subscriber rightImgSub_;

  //sensor_msgs::Image img_left_;
  //sensor_msgs::Image img_right_;

  cv::Mat img_left_;
  cv::Mat img_right_;

  bool img_left_flag_;
  bool img_right_flag_;

  int img_width_;
  int img_height_;

};
