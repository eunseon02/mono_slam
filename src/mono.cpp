
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("image_publisher");

    // Create a publisher to publish sensor_msgs::msg::Image
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("/camera/raw_image", 10);

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("/camera/raw_image", 10)
    
    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);


        // Pass the image to the SLAM system
        // SLAM.TrackMonocular(im);

        if (im.empty())
        {
            cerr << endl << "Failed to load image at: "
                << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // Create a sensor_msgs::msg::Image message
        sensor_msgs::msg::Image msg;
        msg.header.stamp = rclcpp::Clock().now();  // Set the message timestamp
        msg.height = im.rows;
        msg.width = im.cols;
        msg.encoding = "bgr8"; // Assuming the image is in BGR format
        msg.is_bigendian = false;
        msg.step = im.cols * im.elemSize();
        size_t size = msg.step * im.rows;
        msg.data.resize(size);
        memcpy(&msg.data[0], im.data, size);

        // Publish the message
        publisher->publish(msg);

        rclcpp::spin_some(node);  // Process any pending ROS messages

    }

    rclcpp::shutdown();

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}