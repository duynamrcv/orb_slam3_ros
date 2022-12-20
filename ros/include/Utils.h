#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <orb_slam3_ros/dynamic_reconfigureConfig.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "System.h"
#include "ImuTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"

extern ORB_SLAM3::System::eSensor sensor_type;
extern std::string world_frame_id, base_frame_id, cam_frame_id, imu_frame_id;

extern ros::Publisher pose_pub, all_pose_pub, map_points_pub, tracked_map_points_pub;

// initialization Transform listener
extern boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
extern boost::shared_ptr<tf2_ros::TransformListener> tfListener;// Initialization transformation listener


void setupPublishers(ros::NodeHandle&, image_transport::ImageTransport&, ORB_SLAM3::System::eSensor);

void publishMapPoints(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);
void publishTrackedMapPoints(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);
void publishPosition(Sophus::SE3f, ros::Time);
void publishAllPosition(std::vector<ORB_SLAM3::KeyFrame*>, ros::Time);

tf2::Transform transformToTarget(tf2::Transform, std::string, std::string, ros::Time);
tf2::Transform SE3fToTfTransform(Sophus::SE3f);
sensor_msgs::PointCloud2 mapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*>, ros::Time);


#endif