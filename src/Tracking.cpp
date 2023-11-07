#include "Visualization.hpp"

#include <rclcpp/rclcpp.hpp>
// #include <Eigen/Core>
// #include <opencv2/core/eigen.hpp>

#include<iostream>
#include<fstream>

#include "Tracking.hpp"


using namespace std;
using namespace cv;

unsigned long Tracking::nextId = 0;

Tracking::Tracking():Node("tracking") {
    // ORB_extractor object 생성
    mpORBextractor = std::make_shared<ORB_extractor>();

    this->pkg_directory = "/home/eunseon/ros2_ws/src/mono_slam/data/images";
    
    RawImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/raw_image", 10);
    PeocessedImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/processed_image", 10);

    // visualization 생성
    mpVisualization = std::make_shared<Visualization>();

    unsigned long nextId = 1; 

    mpCamera = std::make_shared<Camera>();
    mK = EigenMatToCvMat(mpCamera->K());
    
    

}

void Tracking::Run()
{
    RCLCPP_INFO(get_logger(), "start tracking");

    // auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    //     "/camera/image_raw", 10,
    //     std::bind(&Tracking::GrabImage, this, std::placeholders::_1));

    std::vector<sensor_msgs::msg::Image::SharedPtr> imageList = readImagesFromFolder(this->pkg_directory);
    RCLCPP_INFO(get_logger(), "tracking");   

    for (const auto& image : imageList) {
        RawImagePublisher->publish(*image);
        RCLCPP_INFO(get_logger(), "Published image");
        GrabImage(image);
        RCLCPP_INFO(get_logger(), "Return");
    }


    // size_t imageIndex = 0; // 이미지 인덱스 초기화

    // auto timerCallback = [this, imageList, &imageIndex](rclcpp::TimerBase& timer) {
    //     if (imageIndex < imageList.size()) {
    //         // 이미지 발행
    //         RawImagePublisher->publish(*imageList[imageIndex]);
    //         RCLCPP_INFO(get_logger(), "Published image %zu", imageIndex);

    //         // 다음 이미지로 이동 또는 처음 이미지로 순환
    //         imageIndex = (imageIndex + 1) % imageList.size();
    //     }
    // };

    // // 0.3s timer
    // rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
    //     std::chrono::milliseconds(30), timerCallback);

    // std::string node_name = "tracking_node_" + std::to_string(imageIndex);
    // rclcpp::spin(std::make_shared<Tracking>(node_name));    
    
}

void Tracking::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // initialize
    Frame* f = new Frame();
    f->mdId = nextId++;
    RCLCPP_INFO(get_logger(), "%d", nextId);
    // vector<KeyPoint> keypoints1;
    // vector<KeyPoint> keypoints2;
    // vector<DMatch> mvIniMatches;

    RCLCPP_INFO(get_logger(), "GrabImage");
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("tracking"), "cv_bridge exception: %s", e.what());
        return;
    }

    assert(cv_ptr->image.channels() == 3 || cv_ptr->image.channels() == 1);

    if(cv_ptr->image.channels() == 3)
    {
        cvtColor(cv_ptr->image, mCurrentImage, CV_RGB2GRAY);
    }
    else if(cv_ptr->image.channels() == 1)
    {
        cv_ptr->image.copyTo(mCurrentImage);
    }

    // if (!(mCurrentFrame->img_).empty()) {
    //     // delete mCurrentFrame->img_;
    //     mCurrentFrame->img_ = (mCurrentFrame->img_).empty(); // 포인터를 null로 설정하여 더 이상 접근하지 않도록 합니다.
    // }
    RCLCPP_INFO(get_logger(), "GrabImage2");


    // ORB extractor for initialization
    if (!mLastImage.empty()) {
        RCLCPP_INFO(get_logger(), "GrabImage3");
        //mCurrentFrame->img_ = im;
        f->img_ = mCurrentImage;
       
        mpORBextractor->extractAndMatchORB(mLastImage, f->img_, mLastKeypoint, f->keypoints, f->fIniMatches);

        //
        // /*
        // 매칭결과
        // -> 매칭되면
        // -> 맵포인트 불러와서
        // -> 그 맵포인트에 Observation 추가
        // -> 그 맵포인트를 현재 프레임에 넣어줘야됨
        // -> 매칭 안됨 -> 생성
        // */

        // Frame f = new;
        
        // Mappoint *m = new MapPoint();
        // m->mObservations.insert(f);
        
        RCLCPP_INFO(get_logger(), "GrabImage5");

        // return;


    }
    else{
        mLastImage = mCurrentImage.clone();
        // mLastFrame->img_ = im.clone(); // 1st frame - last frame matching
        // mLastFrame->keypoint=extractORB(mLastFrame, keypoints)

        // origin of map
        RCLCPP_INFO(get_logger(), "GrabImage4");
        // mpORBextractor->extractAndMatchORB(mLastFrame->img_, im, mLastFrame->keypoint, keypoints, mvIniMatches);
    
        f->img_ = mCurrentImage;
        mpORBextractor->extractORB(f->img_, f->keypoints);

        return;

    }

    
    // // update Last Image
    // mLastImage = im.clone();
    // mCurrentFrame = std::make_shared<Frame>(im, cv_ptr->header.stamp.sec, mpORBextractor);


    // mLastFrame = mCurrentFrame;  // copy

    mLastImage = f->img_;
    mLastKeypoint = f->keypoints;


    RCLCPP_INFO(get_logger(), "GrabImage6");

    TrackPreviousFrame(f);

    RCLCPP_INFO(get_logger(), "GrabImage7");

    // insert a keyframe
    if(NeedNewKeyFrame()) {
        // current frame is a new keyframe
        f->SetKeyFrame();
        mnLastKeyFrameId = f->mdId;
    }



    return;


    // // initial pose estimation


    // mpVisualization->VisualizeCamera(mCurrentFrame);

    // // Track Frame.
    // bool bOK;
    
    // // If we have an initial estimation of the camera pose and matching. Track the local map.
    // bOK = TrackLocalMap(); 




    // return;
}
//chatgpt
std::vector<sensor_msgs::msg::Image::SharedPtr> Tracking::readImagesFromFolder(const std::string& folderPath) {
    std::vector<sensor_msgs::msg::Image::SharedPtr> images;

    // Open the folder
    cv::String folder = folderPath + "/*.jpg";
    std::vector<cv::String> fn;
    cv::glob(folder, fn, false);

    for (const auto& filename : fn) {
        // Read the image using OpenCV
        cv::Mat cvImage = cv::imread(filename, cv::IMREAD_COLOR);
        // cv::imshow("mono_slam", cvImage);
        if (!cvImage.empty()) {
            RCLCPP_INFO(get_logger(), "Image loaded successfully: %s", filename.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to load image: %s", filename.c_str());
        }

        // Convert the OpenCV image to a ROS sensor_msgs::msg::Image
        sensor_msgs::msg::Image::SharedPtr rosImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cvImage).toImageMsg();

        images.push_back(rosImage);
    }
 
    return images;
}

void Tracking::TrackPreviousFrame(Frame* f)
{

    RCLCPP_INFO(get_logger(), "TrackPreviousFrame");

    Mat Rcw; // Current Camera Rotation
    Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    vector<Point3d> points; //generate map point

    // Estimate the motion between two frames
    PoseEstimation(mLastKeypoint , f->keypoints, mvIniMatches, Rcw, tcw);

    f->SetmRcw(Rcw);
    f->Setmtcw(tcw);

    RCLCPP_INFO(get_logger(), "TrackPreviousFrame!");

    // Triangulation(mLastFrame->keypoints, f->keypoints, mvIniMatches, Rcw, tcw, points);

}

void Tracking::PoseEstimation(vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches, Mat &R, Mat &t) {


    RCLCPP_INFO(get_logger(), "PoseEstimation");   
    // Camera Intrinsics
    Mat K = mK;

    // K 유효성
    RCLCPP_ERROR(get_logger(), "K.rows: %d", K.rows);
    RCLCPP_ERROR(get_logger(), "K.cols: %d", K.cols);


    //−− Convert the matching point to the form of vector<Point2f>
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    if (points1.size() < 5 || points2.size() < 5) {
        RCLCPP_ERROR(get_logger(), "Insufficient point pairs for essential matrix estimation. At least 5 pairs are required.");
        // Handle the error as needed
        return;
    }
  

    //−− Calculate essential matrix
    Point2d principal_point(K.at<double>(0, 2), K.at<double>(1, 2)); // camera principal point
    double focal_length = K.at<double>(0, 0); // camera focal length
    RCLCPP_INFO(get_logger(), "PoseEstimation2");  
    Mat essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    // cout << "essential_matrix is " << endl << essential_matrix << endl;



    RCLCPP_INFO(get_logger(), "PoseEstimation3");   


    //−− Recover rotation and translation from the essential matrix.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    RCLCPP_ERROR(get_logger(), "Estimated R: %f %f %f", R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2));
    RCLCPP_ERROR(get_logger(), "Estimated R: %f %f %f", R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2));
    RCLCPP_ERROR(get_logger(), "Estimated R: %f %f %f", R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    RCLCPP_ERROR(get_logger(), "Estimated t: %f %f %f", t.at<double>(0), t.at<double>(1), t.at<double>(2));



    // cout << "R is " << endl << R << endl;
    // cout << "t is " << endl << t << endl;

}

void Tracking::Triangulation(const vector<KeyPoint> keypoints_1, const vector<KeyPoint> keypoints_2, const std::vector<DMatch> &matches, const Mat &R, const Mat &t, vector<Point3d> &points) {
    Mat T1 = (Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
    Mat T2 = (Mat_<float>(3, 4) <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );

    // Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point2f> pts_1, pts_2;
    for (DMatch m:matches) {
    // Convert pixel coordinates to camera coordinates
    pts_1.push_back(Projection(keypoints_1[m.queryIdx].pt, mK));
    pts_2.push_back(Projection(keypoints_2[m.trainIdx].pt, mK));
    }

    Mat pts_4d;
    triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    // Convert to non−homogeneous coordinates
    for (int i = 0; i < pts_4d.cols; i++) {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); // � � �
    Point3d p(
    x.at<float>(0, 0),
    x.at<float>(1, 0),  
    x.at<float>(2, 0)
    );
    points.push_back(p);
    }
}

Point2d Tracking::Projection(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void Tracking::ConvertToPose(const cv::Mat& Rcw, const cv::Mat& tcw) {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    cv::cv2eigen(Rcw, R);
    cv::cv2eigen(tcw, t);

    Sophus::SE3d se3(R, t);

    mCurrentFrame->SetPose(se3);

    // mCurrentFrame->pose_ = se3;
}

bool Tracking::NeedNewKeyFrame() {



    return true;
    // // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // const bool c1 = mCurrentFrame->mdId>=mnLastKeyFrameId+mMaxFrames;
    // const bool c2 = mnMatchesInliers<nRefMatches*0.9 && mnMatchesInliers>15;

    // if(c1&&c2)
    // {
    //     if(LocalMappingState)
    //     {
    //         return true;
    //     }
    //     else
    //     {
    //         /////stop local mapping thread 
    //         return false;
    //     }

    // }
    // else
    //     return  false;
}

// bool Tracking::TrackLocalMap()
// {
//     //projection

//     // case1.


// }

cv::Mat Tracking::EigenMatToCvMat(const Eigen::Matrix<double, 3, 3>& eigenMat)
{
    cv::Mat cvMat(3, 3, CV_64F); // 3x3의 double 형식의 cv::Mat를 생성
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cvMat.at<double>(i, j) = eigenMat(i, j); // Eigen Matrix의 각 요소를 cv::Mat로 복사
        }
    }
    return cvMat;
}