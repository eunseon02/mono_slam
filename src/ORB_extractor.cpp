#include <opencv2/core/core.hpp>    
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>

#include "ORB_extractor.hpp"


using namespace std;
using namespace cv;

ORB_extractor::ORB_extractor(): Node("ORB_extractor") {  
    MatchingResultPublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/matching", 10);
    Raw_ImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/raw_image_check", 10);
}

void ORB_extractor::extractAndMatchORB(const cv::Mat &img_1, const cv::Mat &img_2, vector<KeyPoint> &keypoints_1, vector<KeyPoint> &keypoints_2, vector<DMatch> &matches) {
    // if (argc != 3) {그
    //     cout << "usage: feature_extraction img1 img2" << endl;
    //     return 1;
    // }
    // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ORB_extractor");

    // std::string img1_path, img2_path;
    // node->declare_parameter<std::string>("img1_path", "/path/to/default_img1.jpg");
    // node->declare_parameter<std::string>("img2_path", "/path/to/default_img2.jpg");

    // node->get_parameter("img1_path", img1_path);
    // node->get_parameter("img2_path", img2_path);

    // //−− read images
    // Mat img_1 = imread(img1_path, IMREAD_COLOR);
    // Mat img_2 = imread(img2_path, IMREAD_COLOR);
    // assert(img_1.data != nullptr && img_2.data != nullptr);

    // RCLCPP_INFO(get_logger(), "extractAndMatchORB");

    //−− initialization
    Mat descriptors_1, descriptors_2;
    // RCLCPP_INFO(get_logger(), "extractAndMatchORB");awq2                     
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    RCLCPP_INFO(get_logger(), "extractAndMatchORB");

    //−− detect Oriented FAST
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // vector<KeyPoint> keypoints_1;

    // if (img_1.empty() || img_2.empty() || keypoints_1.empty() || keypoints_2.empty()) {
    //     RCLCPP_ERROR(get_logger(), "Invalid image or keypoints data.");
    //     return; // or handle the error as needed
    // }

    sensor_msgs::msg::Image::SharedPtr img1_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_1).toImageMsg();
    Raw_ImagePublisher->publish(*img1_msg);

    RCLCPP_INFO(get_logger(), "drawMatches");   
    // if (keypoints_1.empty()){
    //     detector->detect(img_1, keypoints_1);    
    //     descriptor->compute(img_1, keypoints_1, descriptors_1);
    // }
    
    //--
    detector->detect(img_1, keypoints_1);    
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    //--

    detector->detect(img_2, keypoints_2);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    
    RCLCPP_INFO(get_logger(), "drawMatches2");   

    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // imshow("ORB features", outimg1);

    RCLCPP_INFO(get_logger(), "drawMatches3"); 

    //−− use Hamming distance to match the features
    // t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    // t2 = chrono::steady_clock::now();
    // time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

    RCLCPP_INFO(get_logger(), "drawMatches3"); 

    //−− sort and remove the outliers
    // min and max distance
    auto min_max = minmax_element(matches.begin(), matches.end(), [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    // printf("-- Max dist : %f \n", max_dist);
    // printf("-- Min dist : %f \n", min_dist);

    // remove the bad matching
    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
    if (matches[i].distance <= max(2 * min_dist, 30.0)) {
        good_matches.push_back(matches[i]);
        }
    }
    //−− draw the results
    // Mat img_match;
    Mat img_goodmatch;
    // drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    // RCLCPP_INFO(get_logger(), "drawMatches");

    RCLCPP_INFO(get_logger(), "drawMatches!!"); 

    // Inside the extractAndMatchORB function
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_goodmatch);
    sensor_msgs::msg::Image img_msg;
    img_bridge.toImageMsg(img_msg);

    // Publish the image message
    MatchingResultPublisher->publish(img_msg);

    // MatchingResultPublisher->publish(img_goodmatch);
    // imshow("all matches", img_match);
    // imshow("good matches", img_goodmatch);

    
    // waitKey(0);

}

void ORB_extractor::extractORB(const cv::Mat &img, vector<KeyPoint> &keypoints) {
    Mat descriptors_;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    detector->detect(img, keypoints);
    descriptor->compute(img, keypoints, descriptors_);


    RCLCPP_INFO(get_logger(), "extractORB");   


    // Mat outimg;
    // drawKeypoints(img, keypoints, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
}