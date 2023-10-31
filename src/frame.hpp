#ifndef FRAME_H
#define FRAME_H

#include "frame.hpp"

#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "ORB_extractor.hpp"
#include "camera.hpp"
#include "Visualization.hpp"

using namespace std;
using namespace cv;

class Frame {
public:
    Frame();
    // ~Frame();

    typedef std::shared_ptr<Frame> Ptr;
    unsigned long mdId = 0;  // id of this frame
    unsigned long mdLastKeyframeId = 0; //id of last keyframe
    unsigned long mdKeyframeId = 0; //id of keyframe
    bool isKeyframe = false; //keyframe
    double time_stamp_;
    SE3 pose_;
    std::mutex pose_mutex_;
    cv::Mat img_;
    // extracted features in image
    vector<KeyPoint> keypoint;

    std::shared_ptr<ORB_extractor> mpORBextractor;  

    // Camera Pose
    cv::Mat mTcw;
    void UpdatePoseMatrices();
//--
    Frame(Mat &im, const double &timeStamp, std::shared_ptr<ORB_extractor> extractor);

    cv::Mat mDistCoef;

    // set and get pose, thread safe
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// Set keyframe and keyframe_id
    void SetKeyFrame(){
        static long KeyframeFactorId = 0;
        isKeyframe = true;
        mdKeyframeId = KeyframeFactorId++;
    }

private:
    // Call UpdatePoseMatrices(), before using
    cv::Mat mOw;
    cv::Mat mRcw;
    cv::Mat mtcw;



};
#endif