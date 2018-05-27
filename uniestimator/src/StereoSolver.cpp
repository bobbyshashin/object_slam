#include "StereoSolver.h"

using namespace cv;
using namespace cv::xfeatures2d;

sensor_msgs::Image StereoSolver::cvMatToROSMsg(const Mat image, const std::string frame_id, const std::string encoding){

    cv_bridge::CvImage img;
    img.image = image;
    img.header.frame_id = frame_id;
    img.encoding = encoding;
    return *img.toImageMsg();
}

Mat StereoSolver::ROSMsgToCvMat(const sensor_msgs::Image msg){

    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image;

}


void StereoSolver::stitchImg(){
    if(!(img_left_flag_ && img_right_flag_))
        return;

    Mat img_stitched;
    hconcat(img_left_, img_right_, img_stitched);

    batchImgPub_.publish(cvMatToROSMsg(img_stitched));

    img_left_flag_ = false;
    img_right_flag_ = false;

}

void StereoSolver::publishBatchImg(){

    if(img_left_flag_ && img_right_flag_){

        batchImgPub_.publish(cvMatToROSMsg(img_left_));
        usleep(10*1000);
        batchImgPub_.publish(cvMatToROSMsg(img_right_));
        img_left_flag_ = false;
        img_right_flag_ = false;
        //std::cout << "Image batch published!" << std::endl;
    }
    else
        std::cout << "No matching images!" << std::endl;
}

bool StereoSolver::featureMatching(){

    if(!img_left_.data || !img_right_.data){
        ROS_ERROR("Error reading images!");
        return false;
    }

    int minHessian = 400;
    Ptr<SIFT> detector = SIFT::create(minHessian);

    std::vector<KeyPoint> keypoints_left, keypoints_right;
    detector->detect(img_left_, keypoints_left);
    detector->detect(img_right_, keypoints_right);

    Mat img_keypoints_left, img_keypoints_right;

    Ptr<SiftDescriptorExtractor> extractor = SiftDescriptorExtractor::create();

    Mat descriptor_left, descriptor_right;

    extractor->compute(img_left_, keypoints_left, descriptor_left);
    extractor->compute(img_right_, keypoints_right, descriptor_right);

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

    drawMatches(img_left_, keypoints_left, img_right_, keypoints_right, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    imshow("Matched Keypoints", img_matches);

    std::cout << "Matched keypoints: " << (int)good_matches.size() << std::endl;

    // drawKeypoints(img_left, keypoints_left, img_keypoints_left, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // drawKeypoints(img_right, keypoints_right, img_keypoints_right, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    // imshow("Keypoints Left", img_keypoints_left);
    // imshow("Keypoints Right", img_keypoints_left);

    waitKey(10);

    return true;
    



}


void StereoSolver::leftImgCallback(const sensor_msgs::ImageConstPtr& msg){

    img_left_ = ROSMsgToCvMat(*msg);
    img_left_flag_ = true;
    //stitchImg();
    publishBatchImg();

}

void StereoSolver::rightImgCallback(const sensor_msgs::ImageConstPtr& msg){

    img_right_ = ROSMsgToCvMat(*msg);
    img_right_flag_ = true;
    //stitchImg();
    publishBatchImg();

}

void StereoSolver::boundingBoxCallback(const darknet_ros_msgs::BoundingBoxes bboxes){





}