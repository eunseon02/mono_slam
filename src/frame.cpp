#include "frame.hpp"
#include <rclcpp/rclcpp.hpp>

Frame::Frame(cv::Mat &im, const double &timeStamp, std::shared_ptr<ORB_extractor> extractor)
    : mdId(0), mdLastKeyframeId(0), mdKeyframeId(0), isKeyframe(false), time_stamp_(timeStamp), img_(im), mpORBextractor(extractor)
{
   
}
// Frame::~Frame(){
//     for(auto it = mMapPoints.begin(); it != mMapPoints.end; it++)//맵포인트를 하나 씩 가져옴
//     {
//         auto ob = it->mObservations;
//         for(auto itObs)
//         {
//             itObs->erase(this*);
//         }
//     }
    

// }

Frame::Frame(Mat &im, unsigned long mdId): mdId(mdId), mdLastKeyframeId(0), mdKeyframeId(0), isKeyframe(false), img_(im)
{}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

void Frame::UpdatetoWorld()
{

    // for (const auto& curr_point : mCurrentMappoints) {

    //     cv::Point3d point;
        
    //     point.x = mRcw.at<double>(0, 0) * 1.0 + mtcw.at<double>(0, 0);
    //     point.y = mRcw.at<double>(1, 0) * 1.0 + mtcw.at<double>(1, 0);
    //     point.z = mRcw.at<double>(2, 0) * 1.0 + mtcw.at<double>(2, 0);
    //     WorldMappoint.push_back(point);
    // }

    // RCLCPP_INFO(rclcpp::get_logger("LOGGER"), "WorldMappoint :%d", WorldMappoint.size());

    std::vector<cv::Point3d> worldPoints;

    // 카메라 좌표계에서 월드 좌표계로의 변환 행렬 계산
    cv::Mat transformMatrix = cv::Mat::eye(4, 4, CV_64F);
    mRcw.copyTo(transformMatrix(cv::Rect(0, 0, 3, 3)));
    mtcw.copyTo(transformMatrix(cv::Rect(3, 0, 1, 3)));

    // Homogeneous 좌표계로 변환
    cv::Mat homogeneousPoint(4, 1, CV_64F);
    homogeneousPoint.at<double>(3, 0) = 1.0;

    for (const auto& cameraPoint : mCurrentMappoints) {
        homogeneousPoint.at<double>(0, 0) = cameraPoint.x;
        homogeneousPoint.at<double>(1, 0) = cameraPoint.y;
        homogeneousPoint.at<double>(2, 0) = cameraPoint.z;

        // 카메라 좌표계의 점을 월드 좌표계로 변환
        cv::Mat worldPoint = transformMatrix * homogeneousPoint;
        // WorldMappoint.emplace_back(worldPoint.at<double>(0, 0), worldPoint.at<double>(1, 0), worldPoint.at<double>(2, 0));
        WorldMappoint.emplace_back(worldPoint.at<double>(2, 0), worldPoint.at<double>(0, 0), worldPoint.at<double>(1, 0));
    }


}

