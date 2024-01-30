#include "Visualization.hpp"

#include <rclcpp/rclcpp.hpp>
// #include <Eigen/Core>
// #include <opencv2/core/eigen.hpp>

#include<iostream>
#include<fstream>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"


#include "Tracking.hpp"


using namespace std;
using namespace cv;

unsigned long Tracking::nextId = 0;
vector<Point2f> getGreyCamGroundPoses();


Tracking::Tracking():Node("tracking") {
    // ORB_extractor object 생성
    mpORBextractor = std::make_shared<ORB_extractor>();

    this->pkg_directory = "/home/eunseon/ros2_ws/src/mono_slam/data/images";
    
    RawImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/raw_image", 10);
    PeocessedImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/processed_image", 10);
    test_keypoint_Publisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/test", 10);
    publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_arr", 10);
    pub = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    mappooint_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/mappoint", 10);
    mappooint_publisher2 = this->create_publisher<visualization_msgs::msg::MarkerArray>("/mappoint2", 10);
    // publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_topic", 10);


    // visualization 생성
    mpVisualization = std::make_shared<Visualization>();

    unsigned long nextId = 1; 

    mpCamera = std::make_shared<Camera>();
    mK = EigenMatToCvMat(mpCamera->K());


    dataset_images_location = "/home/eunseon/ros2_ws/src/mono_slam/dataset/KITTI/data_odometry_gray/dataset/sequences/00/image_1";
    dataset_poses_location = "/home/eunseon/ros2_ws/src/mono_slam/dataset/KITTI/data_odometry_poses/dataset/poses/00.txt";
    
    groundScales = getAbsoluteScales();
    groundPoses = getGreyCamGroundPoses();

    id = 0;
    mappoint_id = 0;
    

    mMaxFrames = 8;

    mnLastKeyFrameId = 0;
    validation = 1;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    ///////////////////////////////////////////////////////////////////////////////
    
}


void Tracking::Run()
{
    RCLCPP_INFO(get_logger(), "start tracking");

    // // auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    // //     "/camera/image_raw", 10,
    // //     std::bind(&Tracking::GrabImage, this, std::placeholders::_1));

    // std::vector<sensor_msgs::msg::Image::SharedPtr> imageList = readImagesFromFolder(this->pkg_directory);
    // RCLCPP_INFO(get_logger(), "tracking");   

    // for (const auto& image : imageList) {
    //     RawImagePublisher->publish(*image);
    //     // RCLCPP_INFO(get_logger(), "Published image");
    //     GrabImage(image);
    //     // RCLCPP_INFO(get_logger(), "Return");
    // }

    Mat traj = Mat::zeros(600, 1241, CV_8UC3);
//-----------------------
    char filename[200];   
    for(int numFrame=0; numFrame < MAX_FRAME; numFrame++) {
        sprintf(filename, "%s/%06d.png", dataset_images_location, numFrame);
        Mat img_c = cv::imread(filename);
        // Mat img;
        // cvtColor(img_c, img, cv::COLOR_BGR2GRAY); 

        // cv::imshow("test",img_c);
        // cv::waitKey(1);
        GrabImage(img_c, traj, numFrame);
    }
}

void Tracking::GrabImage(Mat msg, Mat traj, int numFrame)
{
    // initialize
    Frame* f = new Frame(msg, nextId++);
    // f->mdId = nextId++;
    RCLCPP_INFO(get_logger(), "%d", f->mdId);
    mCurrentImage = msg.clone();
    
    mCurrentMappoints.clear(); // empty previous frame mappoint
    // RCLCPP_INFO(get_logger(), "clear");
    // RCLCPP_INFO(get_logger(), "mCurrentMappoints size : %d", mCurrentMappoints.size());



    // ORB extractor for initialization
    if (f->mdId==0) {
        // RCLCPP_INFO(get_logger(), "GrabImage3");
        //mCurrentFrame->img_ = im;
        f->img_ = mCurrentImage.clone();

        
        mpORBextractor->extractORB(f->img_ , f->keypoints);
        mLastKeypoint = f->keypoints;
        mLastImage = f->img_;

        // featureTracking(img_1,img_2,points1,points2, f->status); //track those features to img_2 
            
        // mpORBextractor->extractAndMatchORB(mLastImage, f->img_, mLastKeypoint, f->keypoints, f->fIniMatches);

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
        
        // RCLCPP_INFO(get_logger(), "GrabImage5");

        return;


    }
    else{
        // RCLCPP_INFO(get_logger(), "GrabImage4");
        mpORBextractor->extractAndMatchORB(mLastImage,f->img_ ,mLastKeypoint,f->keypoints, f->matches); 

        f->SetKeyFrame();
        // mLastFrame->img_ = im.clone(); // 1st frame - last frame matching
        // mLastFrame->keypoint=extractORB(mLastFrame, keypoints)

        // origin of map

        // mpORBextractor->extractAndMatchORB(mLastFrame->img_, im, mLastFrame->keypoint, keypoints, mvIniMatches);
    
        // return;

    }


    // mLastImage = f->img_;
    // mLastKeypoint = f->keypoints;
 


    // RCLCPP_INFO(get_logger(), "GrabImage6");
    mvIniMatches = f->matches;  
    TrackPreviousFrame(f);

    // RCLCPP_INFO(get_logger(), "frame.mCurrentMappoints size : %d", f->mCurrentMappoints.size());
 
    mLastKeypoint = mCurrentKeypoint;
    mLastImage = mCurrentImage.clone();
    
    // insert a keyframe
    if(NeedNewKeyFrame(f)) {
        // current frame is a new keyframe
        RCLCPP_INFO(get_logger(), "NeedNewKeyFrame");
        f->SetKeyFrame();
        mnLastKeyFrameId = f->mdId;

        RCLCPP_INFO(get_logger(), "frame.WorldMappoint size : %d", f->WorldMappoint.size());
        
        keyframes.push_back(f);
        for (const auto& point_3d : f->WorldMappoint) {
            mappoints.push_back(Point3dToGeometryMsgPoint(point_3d));
        }
        RCLCPP_INFO(get_logger(), "mappoints size : %d", mappoints.size()); 
        publishMapPointVisualization(f);
        // mappoints.insert(mappoints.end(), mCurrentMappoints.begin(), mCurrentMappoints.end());
    }

    // VisualizeCamera(f)

    //---
    // make result window
    // cv::Point3d p(tcw.at<double>(0), tcw.at<double>(1), tcw.at<double>(2));
    // geometry_msgs::msg::Point point;
    // point.x = p.z;
    // point.y = p.x;
    // point.z = p.y;
    // PointVisualization(point);

    // RCLCPP_INFO(get_logger(), "mappoint:%d", mappoints.size());
    // RCLCPP_INFO(get_logger(), "mCurrentMappoints:%d", mCurrentMappoints.size());


    publishPointVisualization(f);

    publishMapPointVisualization2();
    // publishMapPointVisualization();

    // RCLCPP_INFO(get_logger(), "last keyframe : %d", mnLastKeyFrameId);
    // cout << path.size() << '\n';

    // RCLCPP_INFO(get_logger(), "Estimated t: %f %f %f", tcw.at<double>(0), tcw.at<double>(1), tcw.at<double>(2));

//-------------map---------------


    if (f->mdId<=7 && (f->mdId%2==0)) {
        for (const auto& point_3d : f->WorldMappoint) {
            mappoints.push_back(Point3dToGeometryMsgPoint(point_3d));
        }
    }



//
    cv::namedWindow( "Road facing camera | Top Down Trajectory", cv::WINDOW_AUTOSIZE);// Create a window for display.

    // Mat traj = Mat::zeros(376, 1241, CV_8UC3);

    Mat show = msg;
    int x = int(tcw.at<double>(0)) + 600;
    int y = int(-(tcw.at<double>(2))) + 100;

    // for (const auto& point : f->mCurrentMappoints) {
    //     // x, y 좌표 추출
    //     int x1 = static_cast<int>(point.x);
    //     int y1 = static_cast<int>(point.y);

    //     // 이미지에 점 그리기
    //     cv::circle(traj, cv::Point(x1, y1), 1, CV_RGB(255, 0, 0), 2);
    // }
    circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);
    circle(traj, cv::Point(groundPoses[numFrame].x+600, groundPoses[numFrame].y+100), 1, CV_RGB(0, 255, 0), 2);

    rectangle(traj, cv::Point(10, 30), cv::Point(550, 50), cv::Scalar(0, 0, 0), cv::FILLED);
    // sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", tcw.at<double>(0), tcw.at<double>(1),tcw.at<double>(2));
    // putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

    if (true){
      //Draw features as markers for fun
      for(auto point: mCurrentKeypoint)
        cv::drawMarker(show, point.pt, CV_RGB(0,255,0), cv::MARKER_TILTED_CROSS, 2, 1, cv::LINE_AA);
    }



    Mat concated;
    cv::vconcat(show, traj, concated);

    imshow("Road facing camera | Top Down Trajectory", concated);


    cv::waitKey(1);



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
// keypoints
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

    // RCLCPP_INFO(get_logger(), "TrackPreviousFrame");
// 
    Mat R; // Current Camera Rotation
    Mat t; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    vector<Point3d> points; //generate map point

    scale = groundScales[f->mdId];

    // cv::imshow("track", f->img_)

    // Estimate the motion between two frames
    PoseEstimation(mLastKeypoint , f->keypoints, mvIniMatches, R, t);
    // RCLCPP_INFO(get_logger(), "%d", f->mdId);
    if(f->mdId == 1){

        // RCLCPP_INFO(get_logger(), "1");
        f->SetmRcw(R);
        f->Setmtcw(t);  
        
        Rcw = R.clone();
        tcw = t.clone();

    }
    else{
        tcw = tcw + scale * (Rcw * t);
        Rcw = R * Rcw;
        // RCLCPP_ERROR(get_logger(), "2");
        
        f->SetmRcw(Rcw);
        f->Setmtcw(tcw);
    }

    



    Triangulation(f, mLastKeypoint, f->keypoints, f->matches, R, t, f->mCurrentMappoints);
    RCLCPP_INFO(get_logger(), "frame.mCurrentMappoints size : %d", f->mCurrentMappoints.size());

    // Triangulation(mLastKeypoint, f->keypoints, f->matches, R, t, points);
    f->UpdatetoWorld();

}



void Tracking::PoseEstimation(vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches, Mat &R, Mat &t) {

    // RCLCPP_INFO(get_logger(), "PoseEstimation");   


    // test-------------------------------                                                 ----
    // // keypoint test
    // Mat outimg1;
    // drawKeypoints(mLastImage, keypoints_2, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    // // imshow("ORB features", outimg1);

    // sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outimg1).toImageMsg();

    // // Publish the ROS image message
    // test_keypoint_Publisher->publish(*msg);
    //------------------------
    // // //test match
    // Mat img_goodmatch;
    // // drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    // drawMatches(mLastImage, keypoints_1, mCurrentImage, keypoints_2, matches, img_goodmatch);
    // cv::imshow("match", img_goodmatch);

    // Inside the extractAndMatchORB function
    // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_goodmatch);
    // sensor_msgs::msg::Image img_msg;
    // img_bridge.toImageMsg(img_msg);

    // Publish the image message
    // test_keypoint_Publisher->publish(img_msg);

    // //----

    // Camera Intrinsics
    Mat K = mK;

    //−− Convert the matching point to the form of vector<Point2f>
    vector<Point2f> points1;
    vector<Point2f> points2;

    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    // std::cout << "match num: " << matches.size() << "\n";
    // RCLCPP_ERROR(get_logger(),"match num: %d", matches.size());
    if (points1.size() < 5 || points2.size() < 5) {
        RCLCPP_ERROR(get_logger(), "Insufficient point pairs for essential matrix estimation. At least 5 pairs are required.");
        // Handle the error as needed
        return;
    }
  
    //−− Calculate essential matrix
    Point2d principal_point(K.at<double>(0, 2), K.at<double>(1, 2)); // camera principal point
    double focal_length = K.at<double>(0, 0); // camera focal length
    // RCLCPP_INFO(get_logger(), "PoseEstimation2");  
    Mat essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point, RANSAC, 0.999, 1.0);
    // cout << "essential_matrix is " << endl << essential_matrix << endl;

    // RCLCPP_INFO(get_logger(), "PoseEstimation3");   

    //−− Recover rotation and translation from the essential matrix.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    // RCLCPP_ERROR(get_logger(), "Estimated R: %f %f %f", R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2));
    // RCLCPP_ERROR(get_logger(), "Estimated R: %f %f %f", R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2));
    // RCLCPP_ERROR(get_logger(), "Estimated R: %f %f %f", R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    // RCLCPP_ERROR(get_logger(), "Estimated t: %f %f %f", t.at<double>(0), t.at<double>(1), t.at<double>(2));






    // cout << "R is " << endl << R << endl;
    // cout << "t is " << endl << t << endl;

}

void Tracking::Triangulation(Frame* f, const vector<KeyPoint> keypoints_1, const vector<KeyPoint> keypoints_2, const std::vector<DMatch> &matches, const Mat &R, const Mat &t, vector<Point3d> &points) {
    Mat T1 = (Mat_<float>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    Mat T2 = (Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0), R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    //Mat T2 = (Mat_<float>(3, 4) << Rcw.at<double>(0, 0), Rcw.at<double>(0, 1), Rcw.at<double>(0, 2), tcw.at<double>(0, 0), Rcw.at<double>(1, 0), Rcw.at<double>(1, 1), Rcw.at<double>(1, 2), tcw.at<double>(1, 0), Rcw.at<double>(2, 0), Rcw.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    // RCLCPP_INFO(get_logger(), "GrabImage6");
    // Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); 
    vector<Point2f> pts_1, pts_2;
    for (DMatch m:matches) {
        // Convert pixel coordinates to camera coordinates
        pts_1.push_back(Projection(keypoints_1[m.queryIdx].pt, mK));
        pts_2.push_back(Projection(keypoints_2[m.trainIdx].pt, mK));
    }

    Mat pts_4d;
    triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    // RCLCPP_INFO(get_logger(), "GrabImage6");

    // RCLCPP_INFO(get_logger(), "Estimated t: %f %f %f", tcw.at<double>(0), tcw.at<double>(1), tcw.at<double>(2));

    // Convert to non−homogeneous coordinates
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // � � �
        Point3d p(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
        // geometry_msgs::msg::Point point;
        // point.x = p.z;
        // point.y = p.x;
        // point.z = p.y;
        f->mCurrentMappoints.push_back(p);
        // points.push_back(point);
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

bool Tracking::NeedNewKeyFrame(Frame* f)
{
    // RCLCPP_INFO(get_logger(), "%d", f->mdId);
    // return true;
    // // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1 = f->mdId >= mnLastKeyFrameId+mMaxFrames;
    // const bool c2 = mnMatchesInliers<nRefMatches*0.9 && mnMatchesInliers>15;
    cout<<c1;

    // if(c1&&c2)
    if(c1)
    {
        return true;

        // if(LocalMappingState)
        // {
        //     return true;
        // }
        // else
        // {
        //     /////stop local mapping thread 
        //     return false;
        // }

    }
    else
        return  false;
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
vector<double> Tracking::getAbsoluteScales(){
    vector<double> scales;
    string line;
    int i = 0;
    ifstream myfile (dataset_poses_location);
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
    while ( getline (myfile,line)  )
    {
        z_prev = z;
        x_prev = x;
        y_prev = y;
        std::istringstream in(line);
        //cout << line << '\n';
        for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
        }

        scales.push_back(sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)));
    }
    myfile.close();
    }

    return scales;
}

void Tracking::publishPointVisualization(Frame* f) {
    // RCLCPP_INFO(get_logger(), "VISUAL");
    
    cv::Point3d p(tcw.at<double>(0), tcw.at<double>(1), tcw.at<double>(2));
    geometry_msgs::msg::Point point;
    point.x = p.z;
    point.y = p.x;
    point.z = p.y;


    Eigen::Matrix3d eigenR;
    cv::cv2eigen(f->GetmRcw(), eigenR);

    // Eigen Matrix를 Quaternion으로 변환
    Eigen::Quaterniond quaternion(eigenR);

    geometry_msgs::msg::Quaternion quat_msg;

    quat_msg.x = quaternion.x();
    quat_msg.y = quaternion.y();
    quat_msg.z = quaternion.z();
    quat_msg.w = quaternion.w();


    path.push_back(point);
    orientation_list.push_back(quat_msg);
    visualization_msgs::msg::MarkerArray markerArray;


    // cout << "Point - x: " << path[path.size()-1].x << ", y: " << path[path.size()-1].y << ", z: " << path[path.size()-1].z << endl;
    // cout << "prev Point - x: " << path[path.size()-2].x << ", y: " << path[path.size()-2].y << ", z: " << path[path.size()-2].z << endl;
    // cout << "Point - x: " << point.x << ", y: " << point.y << ", z: " << point.z << endl;

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = rclcpp::Node::now();
    t.header.frame_id = "world";
    t.child_frame_id = "odom";
    t.transform.translation.x = point.x;
    t.transform.translation.y = point.y;
    t.transform.translation.z = point.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0); 
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    for (size_t i = 0; i < path.size(); ++i) {
        // RCLCPP_INFO(get_logger(), "VISUAL");
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world"; 
        marker.header.stamp = rclcpp::Node::now();
        marker.ns = "points";
        marker.id = i; 
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = path[i]; 
        marker.pose.orientation.x = orientation_list[i].x;
        marker.pose.orientation.y = orientation_list[i].y;
        marker.pose.orientation.z = orientation_list[i].z;
        marker.pose.orientation.w = orientation_list[i].w;
        marker.scale.x = 0.5; 
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }
////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
    publisher->publish(markerArray);

}



void Tracking::publishMapPointVisualization(Frame* f) {
    // RCLCPP_INFO(get_logger(), "VISUAL");

    visualization_msgs::msg::MarkerArray markerArray;
    RCLCPP_INFO(get_logger(), "rrmappoint:%d", f->mCurrentMappoints.size());
    geometry_msgs::msg::Point point;


    for (size_t i = 0; i < f->mCurrentMappoints.size(); ++i) {
        // RCLCPP_INFO(get_logger(), "VISUAL");
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom"; 
        marker.header.stamp = rclcpp::Node::now();
        marker.ns = "points";
        marker.id = i; 
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        // marker.pose.position = mCurrentMappoints[i];
        point.x = f->mCurrentMappoints[i].x;
        point.y = f->mCurrentMappoints[i].y;
        point.z = f->mCurrentMappoints[i].z;        
        marker.pose.position = point;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5; 
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }

    mappooint_publisher->publish(markerArray);
    RCLCPP_INFO(get_logger(), "Mapping.. : %d", mappoints.size());

}




void Tracking::publishMapPointVisualization2() {
    // RCLCPP_INFO(get_logger(), "VISUAL");

    visualization_msgs::msg::MarkerArray markerArray;
    // RCLCPP_INFO(get_logger(), "rrmappoint:%d", mCurrentMappoints.size());

    

    for (size_t i = 0; i < mappoints.size(); ++i) {
        // RCLCPP_INFO(get_logger(), "VISUAL");
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world"; 
        marker.header.stamp = rclcpp::Node::now();
        marker.ns = "points";
        marker.id = i; 
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = mappoints[i];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5; 
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        markerArray.markers.push_back(marker);
    }

    mappooint_publisher2->publish(markerArray);
    // RCLCPP_INFO(get_logger(), "Mapping.. : %d", mappoints.size());

}


// void Tracking::PointVisualization(const geometry_msgs::msg::Point& point) {
//     // RCLCPP_INFO(get_logger(), "VISUAL");


//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = "odom"; 
//     marker.header.stamp = rclcpp::Node::now();
//     marker.ns = "points";
//     marker.id = id; 
//     marker.type = visualization_msgs::msg::Marker::SPHERE;
//     marker.action = visualization_msgs::msg::Marker::ADD;
//     marker.pose.position = point; 
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 1.0;
//     marker.scale.x = 0.5; 
//     marker.scale.y = 0.5;
//     marker.scale.z = 0.5;
//     marker.color.a = 1.0; 
//     marker.color.r = 1.0;
//     marker.color.g = 0.0;
//     marker.color.b = 0.0;



//     pub->publish(marker);

// }


vector<Point2f> Tracking::getGreyCamGroundPoses() {
    string line;
    int i = 0;
    ifstream myfile (dataset_poses_location);
    double value = 0;
    vector<Point2f> poses;
    if (myfile.is_open())
    {
    while ( getline (myfile,line)  )
    {
        Point2f pose;
        std::istringstream in(line);
        for (int j=0; j<12; j++)  {
        in >> value;
        if (j==11) pose.y=value;
        if (j==3) pose.x=value;
        }

        poses.push_back(pose);
        i++;
    }
    myfile.close();
    }

    return poses;

}


// void Tracking::publishMapPointVisualization()
// {

//     sensor_msgs::msg::PointCloud2 pointcloud_msg;

//     pointcloud_msg.header.frame_id = "point_cloud_frame";
//     pointcloud_msg.height = 1;
//     pointcloud_msg.width = 3; // 총 포인트 수
//     pointcloud_msg.is_bigendian = false;
//     pointcloud_msg.point_step = 16; // 포인트당 바이트 수
//     pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
//     pointcloud_msg.is_dense = true;

//     sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
//     modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
//                                   "y", 1, sensor_msgs::msg::PointField::FLOAT32,
//                                   "z", 1, sensor_msgs::msg::PointField::FLOAT32,
//                                   "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    
//     sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
//     sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
//     sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
//     sensor_msgs::PointCloud2Iterator<float> iter_intensity(pointcloud_msg, "intensity");


//     for (size_t i = 0; i < 3; ++i) {
//         *iter_x = static_cast<float>(i);
//         *iter_y = static_cast<float>(i * 2);
//         *iter_z = static_cast<float>(i * 3);
//         *iter_intensity = static_cast<float>(i * 0.5);

//         ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
//     }
    
//     size_t num_points = mappoints.size();
//     pointcloud_msg.width = num_points;
//     pointcloud_msg.height = 1;
//     pointcloud_msg.point_step = sizeof(float) * 4; // 4개의 float 데이터(x, y, z, intensity)
//     pointcloud_msg.row_step = pointcloud_msg.point_step * num_points;
//     pointcloud_msg.is_dense = true; // 유효한 데이터만 있는 경우 true

//     pointcloud_msg.fields.resize(4);
//     pointcloud_msg.fields[0].name = "x";
//     pointcloud_msg.fields[0].offset = 0;
//     pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
//     pointcloud_msg.fields[0].count = 1;

//     pointcloud_msg.fields[1].name = "y";
//     pointcloud_msg.fields[1].offset = sizeof(float);
//     pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
//     pointcloud_msg.fields[1].count = 1;

//     pointcloud_msg.fields[2].name = "z";
//     pointcloud_msg.fields[2].offset = 2 * sizeof(float);
//     pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
//     pointcloud_msg.fields[2].count = 1;

//     pointcloud_msg.fields[3].name = "intensity";
//     pointcloud_msg.fields[3].offset = 3 * sizeof(float);
//     pointcloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
//     pointcloud_msg.fields[3].count = 1;mappoints

//     pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height);
//     auto float_data = reinterpret_cast<float*>(pointcloud_msg.data.data());
    
//     for (size_t i = 0; i < num_points; ++i) {
//         float_data[i * 4] = point_data[i].x;
//         float_data[i * 4 + 1] = point_data[i].y;
//         float_data[i * 4 + 2] = point_data[i].z;
//         float_data[i * 4 + 3] = point_data[i].intensity;
//     }

//     publisher->publish(pointcloud_msg);
    

// }
geometry_msgs::msg::Point Tracking::Point3dToGeometryMsgPoint(const cv::Point3d& point_3d) {
    geometry_msgs::msg::Point point;
    point.x = point_3d.x;
    point.y = point_3d.y;
    point.z = point_3d.z;
    return point;
}


void Tracking::bundleAdjustmentG2O(Sophus::SE3d &pose) {

    // // const VecVector3d &points_3d,
    // // const VecVector2d &points_2d,
    // // const Mat &K,

    // // 构建图优化，先设定g2o
    // typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
    // typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // // 梯度下降方法，可以从GN, LM, DogLeg 中选
    // auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    // g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    // g2o::SparseOptimizer optimizer;     // 图模型
    // optimizer.setAlgorithm(solver);   // 设置求解器
    // optimizer.setVerbose(true);       // 打开调试输出

    // // vertex
    // VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
    // vertex_pose->setId(0);
    // vertex_pose->setEstimate(Sophus::SE3d());
    // optimizer.addVertex(vertex_pose);

    // // K
    // Eigen::Matrix3d K_eigen;
    // K_eigen <<
    //         K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
    // K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
    // K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // // edges
    // int index = 1;
    // for (size_t i = 0; i < points_2d.size(); ++i) {
    // auto p2d = points_2d[i];
    // auto p3d = points_3d[i];
    // EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
    // edge->setId(index);
    // edge->setVertex(0, vertex_pose);
    // edge->setMeasurement(p2d);
    // edge->setInformation(Eigen::Matrix2d::Identity());
    // optimizer.addEdge(edge);
    // index++;
    // }

    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // optimizer.setVerbose(true);
    // optimizer.initializeOptimization();
    // optimizer.optimize(10);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    // cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
    // pose = vertex_pose->estimate();
}