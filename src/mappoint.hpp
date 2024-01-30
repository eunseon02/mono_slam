#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "mappoint.hpp"

using namespace std;
using namespace cv;

class Mappoint {
public:
    unsigned long id_ = 0; // ID
    Vec3 pos_ = Vec3::Zero(); // pose
    vector<Frame> path;

private:


};
#endif