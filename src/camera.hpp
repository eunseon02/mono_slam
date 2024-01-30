#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "camera.hpp"
#include "common.hpp"

#include <opencv2/opencv.hpp>



using namespace std;
using namespace cv;

// Pinhole stereo camera model
class Camera {
public:
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 7.188560000000e+02, fy_ = 7.188560000000e+02 , cx_ = 6.071928000000e+02, cy_ = 1.852157000000e+02, baseline_ = 0;  // Camera intrinsics
    SE3 pose_;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;         // inverse of extrinsics

    Camera(){};

    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose) : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const { return pose_; }

    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }


};

#endif  // CAMERA_H