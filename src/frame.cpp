#include "frame.hpp"


Frame::Frame(cv::Mat &im, const double &timeStamp, std::shared_ptr<ORB_extractor> extractor)
    : mdId(0), mdLastKeyframeId(0), mdKeyframeId(0), isKeyframe(false), time_stamp_(timeStamp), img_(im), mpORBextractor(extractor)
{
   
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}
