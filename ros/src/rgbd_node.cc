/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
*
*/

#include "Utils.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, bool localize_only):mpSLAM(pSLAM), localize_only(localize_only){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    // void ParamsChangedCallback(orb_slam3_ros::dynamic_reconfigureConfig &config, uint32_t level);

    ORB_SLAM3::System* mpSLAM;

    bool localize_only;
    bool bLocalizationMode = false;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::NodeHandle node_handler;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "/home/nambd/test/orb_slam3_ros_ws/src/orb_slam3_ros/ros/config/ORBvoc.txt");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "/home/nambd/test/orb_slam3_ros_ws/src/orb_slam3_ros/ros/config/astra.yaml");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/base_frame_id", base_frame_id, "base_link");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera_link");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    bool localize_only;
    node_handler.param<bool>(node_name + "/localize_only", localize_only, true);

    // Setup dynamic reconfigure
    dynamic_reconfigure::Server<orb_slam3_ros::dynamic_reconfigureConfig> dynamic_param_server_;
    dynamic_reconfigure::Server<orb_slam3_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::RGBD;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb(&SLAM, localize_only);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/camera/rgb/image_aligned", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/camera/depth/image_aligned", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    setupPublishers(node_handler, image_transport, sensor_type);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    if(!localize_only)
    {
        std::cout << "Save the map " << ORB_SLAM3::System::FileType::BINARY_FILE << std::endl;
        SLAM.SaveAtlas(ORB_SLAM3::System::FileType::BINARY_FILE);
    }
    

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // ORB-SLAM3 runs in TrackRGBD()
    Sophus::SE3f SE3f = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    ros::Time msg_time = cv_ptrRGB->header.stamp;

    // Localization or mapping mode
    if(localize_only && mpSLAM->GetFindLocalization() && !bLocalizationMode)
    {
        mpSLAM->ActivateLocalizationMode();
        bLocalizationMode = true;
    }
    else if(!localize_only && bLocalizationMode)
    {
        mpSLAM->DeactivateLocalizationMode();
        bLocalizationMode = false;
    }

    // Publish topics
    publishAllPosition(mpSLAM->GetKeyFrames(), msg_time);
    publishPosition(SE3f, msg_time);
    publishTrackedMapPoints(mpSLAM->GetTrackedMapPoints(), msg_time);
    publishMapPoints(mpSLAM->GetMapPoints(), msg_time);
}

// void ImageGrabber::ParamsChangedCallback(orb_slam3_ros::dynamic_reconfigureConfig &config, uint32_t level)
// {
//     if(localize_only && !bLocalizationMode)
//     {
//         mpSLAM->ActivateLocalizationMode();
//         bLocalizationMode = true;
//     }
//     else if(!localize_only && bLocalizationMode)
//     {
//         mpSLAM->DeactivateLocalizationMode();
//         bLocalizationMode = false;
//     }
//   mpSLAM->EnableLocalizationOnly (config.localize_only);
//   min_observations_per_point_ = config.min_observations_for_ros_map;

//   if (config.reset_map) {
//     orb_slam_->Reset();
//     config.reset_map = false;
//   }

//   orb_slam_->SetMinimumKeyFrames (config.min_num_kf_in_map);
// }
