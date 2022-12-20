/**
* 
* Common functions and variables across all modes (mono/stereo, with or w/o imu)
*
*/

#include "Utils.h"

ORB_SLAM3::System::eSensor sensor_type;
std::string world_frame_id, base_frame_id, cam_frame_id, imu_frame_id;

ros::Publisher pose_pub, all_pose_pub, map_points_pub, tracked_map_points_pub;

// initialization Transform listener
boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
boost::shared_ptr<tf2_ros::TransformListener> tfListener;// Initialization transformation listener

void setupPublishers(ros::NodeHandle &node_handler,
                    image_transport::ImageTransport &image_transport,
                    ORB_SLAM3::System::eSensor sensor_type)
{
    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>("orb_slam3/camera_pose", 1);
    all_pose_pub = node_handler.advertise<geometry_msgs::PoseArray>("orb_slam3/all_pose_point", 1);
    tracked_map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3/tracked_map_points", 1);
    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3/map_points", 1);

    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));
}

// dynamic_reconfigure::Server<orb_slam3_ros::dynamic_reconfigureConfig> dynamic_param_server_;

void publishPosition(Sophus::SE3f SE3f, ros::Time msg_time)
{
    // Get transform from map to camera frame
    tf2::Transform tf_transform = SE3fToTfTransform(SE3f);
    // Make transform from camera frame to target frame
    tf2::Transform tf_map2target = transformToTarget(tf_transform, cam_frame_id, base_frame_id, msg_time);

    tf2::Stamped<tf2::Transform> tf_map2target_stamped;
    tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, msg_time, world_frame_id);

    // Publish Camera Position As Transform
    geometry_msgs::TransformStamped msg = tf2::toMsg(tf_map2target_stamped);
    msg.child_frame_id = base_frame_id;
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(msg);

    // Publish Camera Position As PoseStamped
    geometry_msgs::PoseStamped pose_msg;
    tf2::toMsg(tf_map2target_stamped, pose_msg);
    pose_pub.publish(pose_msg);
}

void publishAllPosition(std::vector<ORB_SLAM3::KeyFrame*> key_frames, ros::Time msg_time)
{
    geometry_msgs::PoseArray kf_pt_array;
    //! placeholder for number of keyframes
    kf_pt_array.poses.push_back(geometry_msgs::Pose());
    sort(key_frames.begin(), key_frames.end(), ORB_SLAM3::KeyFrame::lId);

    unsigned int n_kf = 0;
    for (auto key_frame : key_frames)
    {
        if(key_frame->isBad()) continue;

        Sophus::SE3f SE3f = key_frame->GetPose();
        tf2::Transform tf_transform = SE3fToTfTransform(SE3f);
        tf2::Transform tf_map2target = transformToTarget(tf_transform, cam_frame_id, base_frame_id, msg_time);
        tf2::Stamped<tf2::Transform> tf_map2target_stamped;
        tf_map2target_stamped = tf2::Stamped<tf2::Transform>(tf_map2target, msg_time, world_frame_id);
        geometry_msgs::Pose pose;
        tf2::toMsg(tf_map2target_stamped, pose);

        kf_pt_array.poses.push_back(pose);
        
        unsigned int n_pts_id = kf_pt_array.poses.size();
        //! placeholder for number of points
        kf_pt_array.poses.push_back(geometry_msgs::Pose());

        std::set<ORB_SLAM3::MapPoint*> map_points = key_frame->GetAllPoints();
        // std::cout << "All Point Size: " << map_points.size() << std::endl;
        unsigned int n_pts = 0;
        for(auto map_pt : map_points)
        {
            if(!map_pt || map_pt->isBad()) continue;

            cv::Mat pt_pose = ORB_SLAM3::Converter::toCvMat(map_pt->GetWorldPos());
            if(pt_pose.empty()) continue;
            geometry_msgs::Pose curr_pt;
            curr_pt.position.x = pt_pose.at<float>(2);
            curr_pt.position.y = -pt_pose.at<float>(0);
            curr_pt.position.z = -pt_pose.at<float>(1);
            kf_pt_array.poses.push_back(curr_pt);
            ++n_pts;
        }
        geometry_msgs::Pose n_pts_msg;
        n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
        kf_pt_array.poses[n_pts_id] = n_pts_msg;
        ++n_kf;
    }
    geometry_msgs::Pose n_kf_msg;
    n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
    kf_pt_array.poses[0] = n_kf_msg;
    kf_pt_array.header.frame_id = "map";
    // kf_pt_array.header.seq = frame_id + 1;
    // printf("Publishing data for %u keyfranmes\n", n_kf);
    all_pose_pub.publish(kf_pt_array);
}

void publishTrackedMapPoints(std::vector<ORB_SLAM3::MapPoint*> tracked_map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mapPointsToPointCloud(tracked_map_points, msg_time);
    tracked_map_points_pub.publish(cloud);
}

void publishMapPoints(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mapPointsToPointCloud(map_points, msg_time);
    map_points_pub.publish(cloud);
}

tf2::Transform transformToTarget(tf2::Transform tf_in, std::string frame_in,
                                std::string frame_out, ros::Time msg_time)
{
    tf2::Transform tf_map2origin = tf_in;
    tf2::Transform tf_origin2target;
    tf2::Transform tf_map2target;

    tf2::Stamped<tf2::Transform> transformStamped_temp;
    try
    {
        // Get the transform from camera to target
        geometry_msgs::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_out, msg_time);

        // Convert to tf2
        tf2::fromMsg(tf_msg, transformStamped_temp);
        tf_origin2target.setBasis(transformStamped_temp.getBasis());
        tf_origin2target.setOrigin(transformStamped_temp.getOrigin());
    }
    catch(tf2::TransformException &ex)
    {
        // ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
        tf_origin2target.setIdentity();
    }
    
    // Transform from map to target
    tf_map2target = tf_map2origin * tf_origin2target;
    return tf_map2target;
}

//
// Miscellaneous functions
//

sensor_msgs::PointCloud2 mapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;
    const int num_channels = 3; // x y z

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for(unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

            tf2::Vector3 point_translation(P3Dw.x(), P3Dw.y(), P3Dw.z());

            float data_array[num_channels] = {
                point_translation.z(),  // x. Do the transformation by just reading at the position of z instead of x
                -point_translation.x(), // y. Do the transformation by just reading at the position of x instead of y
                -point_translation.y()  // z. Do the transformation by just reading at the position of y instead of z
            };

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

tf2::Transform SE3fToTfTransform(Sophus::SE3f T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf2::Matrix3x3 tf_camera_rotation(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );

    tf2::Vector3 tf_camera_translation(
        t_vec(0),
        t_vec(1),
        t_vec(2)
    );

    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                        -1, 0, 0,
                                        0,-1, 0);
    
    //Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

    //Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

    //Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}