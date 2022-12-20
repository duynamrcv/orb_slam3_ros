#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>

using namespace std;
int main(int argc, char** argv)
{
    std::string package = "orb_slam3_ros";
    std::string map_name = "orb_slam3";
    std::string map_path = ros::package::getPath(package) + "/data/"+ map_name +".pgm";

    cv::Mat img = cv::imread(map_path,0);
    cout << img.channels() << endl;
    cv::imwrite("img.png", img);
    
    // cout << int(img.at<uchar>(200,250)) << endl;
    cv::circle(img, cv::Point(250,200),10,cv::Scalar(0,0,0),2);
    cout << int(img.at<uchar>(200, 250)) << endl;
    cv::imshow("img", img);
    cv::waitKey(0);
    return 0;
}