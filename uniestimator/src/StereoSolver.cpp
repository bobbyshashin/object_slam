#include <vector>
//#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBox.h>

#include "StereoSolver.h"

using namespace cv;
using namespace cv::xfeatures2d;

void StereoSolver::stitchImg(){
    if(!(img_left_flag_ && img_right_flag_))
        return;
    
    cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
    try
    {
      cv_ptr_left = cv_bridge::toCvCopy(img_left_, sensor_msgs::image_encodings::BGR8);
      cv_ptr_right = cv_bridge::toCvCopy(img_right_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat img_left, img_right, img_stitched;

    img_left = cv_ptr_left->image;
    img_right = cv_ptr_right->image;

    hconcat(img_left, img_right, img_stitched);
    cv_bridge::CvImage img;
    img.image = img_stitched;
    img.encoding = sensor_msgs::image_encodings::BGR8;

    batchImgPub_.publish(img.toImageMsg());


}

void StereoSolver::publishBatchImg(){

    if(img_left_flag_ && img_right_flag_){

        batchImgPub_.publish(img_left_);
        batchImgPub_.publish(img_right_);
        img_left_flag_ = false;
        img_right_flag_ = false;
        
        std::cout << "Image batch published!" << std::endl;
    }
    else
        std::cout << "No matching images!" << std::endl;
}

bool StereoSolver::featureMatching(){
/*
    if(!img_left.data || !img_right.data){
        ROS_ERROR("Error reading images!");
        return false;
    }

    int minHessian = 400;
    Ptr<SIFT> detector = SIFT::create(minHessian);

    std::vector<KeyPoint> keypoints_left, keypoints_right;
    detector->detect(img_left, keypoints_left);
    detector->detect(img_right, keypoints_right);

    Mat img_keypoints_left, img_keypoints_right;

    Ptr<SiftDescriptorExtractor> extractor = SiftDescriptorExtractor::create();

    Mat descriptor_left, descriptor_right;

    extractor->compute(img_left, keypoints_left, descriptor_left);
    extractor->compute(img_right, keypoints_right, descriptor_right);

    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;

    matcher.match(descriptor_left, descriptor_right, matches);

    double max_dist = 0; double min_dist = 100;

    for( int i = 0; i < descriptor_left.rows; i++ ){ 

        double dist = matches[i].distance;
        if( dist < min_dist ) 
            min_dist = dist;
        if( dist > max_dist ) 
            max_dist = dist;
    }

    std::cout << "-- Max Dist: " << max_dist << std::endl;
    std::cout << "-- Min Dist: " << min_dist << std::endl;

    std::vector<DMatch> good_matches;

    for( int i = 0; i < descriptor_left.rows; i++ ){ 
        if(matches[i].distance <= max(5*min_dist, 0.02))
            good_matches.push_back(matches[i]);
    }

    Mat img_matches;

    drawMatches(img_left, keypoints_left, img_right, keypoints_right, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    imshow("Matched Keypoints", img_matches);

    std::cout << "Matched keypoints: " << (int)good_matches.size() << std::endl;

    // drawKeypoints(img_left, keypoints_left, img_keypoints_left, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // drawKeypoints(img_right, keypoints_right, img_keypoints_right, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    // imshow("Keypoints Left", img_keypoints_left);
    // imshow("Keypoints Right", img_keypoints_left);

    waitKey(10);

    return true;
    */



}
