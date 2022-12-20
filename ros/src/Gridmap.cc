#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <algorithm>
#include <iostream>

// Map parameters
float free_thresh = 0.55;
float occupied_thresh = 0.5;
float scale_factor = 20;

float thresh_max_z, thresh_min_z;
float cloud_max_x, cloud_min_x, cloud_max_y, cloud_min_y;
float grid_max_x, grid_min_x, grid_max_y, grid_min_y;

float norm_factor_x, norm_factor_y;

cv::Mat global_occupied_counter, global_visit_counter;
cv::Mat local_occupied_counter, local_visit_counter;
cv::Mat local_map_pt_mask;
cv::Mat grid_map, grid_map_int, grid_map_thresh;

ros::Publisher gridmap_pub;
nav_msgs::OccupancyGrid grid_map_msg;
sensor_msgs::PointCloud2 tracked_map_point, map_point;
geometry_msgs::PoseStamped camera_pose;
geometry_msgs::PoseArray all_pose_point;

bool loop_closure_being_processed = false;

void allPosePointCallback(const geometry_msgs::PoseArray&);
void cameraPoseCallback(const geometry_msgs::PoseStamped&);
void trackedMapPointCallback(const sensor_msgs::PointCloud2&);
void mapPointCallback(const sensor_msgs::PointCloud2&);
void updateGridMap(const geometry_msgs::PoseStamped&, const sensor_msgs::PointCloud2&);
void resetGridMap(const geometry_msgs::PoseArray&);
void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
	                unsigned int start_id, int pos_grid_x, int pos_grid_y);

void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
	                cv::Mat &visited, cv::Mat &pt_mask, int pos_grid_x, int pos_grid_y);
void getGridMap();
void showGridMap();
bool mapGenerator(const nav_msgs::OccupancyGrid &map_data, const std::string map_name);
bool loadMap(const std::string map_name, nav_msgs::OccupancyGrid& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gridmap");
    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName();

    bool localize_only;
    nh.param<bool>(node_name + "/localize_only", localize_only, true);

    std::string map_name;
    nh.param<std::string>(node_name + "/map_name", map_name, "orb_slam3");
    nh.param<float>(node_name + "/thresh_max_z", thresh_max_z, 0.3);
    nh.param<float>(node_name + "/thresh_min_z", thresh_min_z, -0.1);

    nh.param<float>(node_name + "/cloud_max_x", cloud_max_x, 10.0);
    nh.param<float>(node_name + "/cloud_min_x", cloud_min_x, -10.0);
    nh.param<float>(node_name + "/cloud_max_y", cloud_max_y, 10.0);
    nh.param<float>(node_name + "/cloud_min_y", cloud_min_y, -10.0);
    

    ros::Subscriber map_point_sub = nh.subscribe("orb_slam3/map_points",1, mapPointCallback);
    ros::Subscriber tracked_map_point_sub = nh.subscribe("orb_slam3/tracked_map_points",1, trackedMapPointCallback);
    ros::Subscriber camera_pose_sub = nh.subscribe("orb_slam3/camera_pose",1, cameraPoseCallback);
    ros::Subscriber all_sub = nh.subscribe("orb_slam3/all_pose_point",1, allPosePointCallback);

    ros::Publisher gridmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("orb_slam3/gridmap", 1);

    grid_max_x = cloud_max_x*scale_factor;
    grid_min_x = cloud_min_x*scale_factor;
    grid_max_y = cloud_max_y*scale_factor;
    grid_min_y = cloud_min_y*scale_factor;

    norm_factor_x = float(grid_max_x - grid_min_x - 1) / float(grid_max_x - grid_min_x);
    norm_factor_y = float(grid_max_y - grid_min_y - 1) / float(grid_max_y - grid_min_y);

    // ros::spin();
    ros::Rate r(10);

    if(!localize_only)
    {
        global_occupied_counter.create(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_32SC1);
        global_visit_counter.create(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_32SC1);
        global_occupied_counter.setTo(cv::Scalar(0));
        global_visit_counter.setTo(cv::Scalar(0));

        grid_map_msg.data.resize(int(grid_max_y - grid_min_y)* int(grid_max_x - grid_min_x));
        grid_map_msg.info.width = int(grid_max_x - grid_min_x);
        grid_map_msg.info.height = int(grid_max_y - grid_min_y);
        grid_map_msg.info.resolution = 1.0/scale_factor;
        grid_map_msg.info.origin.position.x = -(cloud_max_x - cloud_min_x)/2;
        grid_map_msg.info.origin.position.y = -(cloud_max_y - cloud_min_y)/2;
        grid_map_msg.info.origin.orientation.w = 1.0;

        grid_map.create(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_32FC1);
        grid_map_int.create(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_8UC1);
        // grid_map = cv::Mat(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_32SC1, (float*)(grid_map_msg.data.data()));
        // grid_map_int = cv::Mat(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_8SC1, (char*)(grid_map_msg.data.data()));

        local_occupied_counter.create(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_32SC1);
        local_visit_counter.create(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_32SC1);
        local_map_pt_mask.create(int(grid_max_y - grid_min_y), int(grid_max_x - grid_min_x), CV_8UC1);

        while (ros::ok())
        {
            resetGridMap(all_pose_point);
            // updateGridMap(camera_pose, tracked_map_point);
            grid_map_msg.info.map_load_time = ros::Time::now();
            grid_map_msg.header.frame_id = "map";
            grid_map_msg.header.stamp = ros::Time::now();

            gridmap_pub.publish(grid_map_msg);

            r.sleep();
            ros::spinOnce();
        }
        while(!mapGenerator(grid_map_msg, map_name)) ros::spinOnce();
    }
    else
    {   
        bool isOK = loadMap(map_name, grid_map_msg);
        std::cout << "Publish gridmap." << std::endl;
        while (ros::ok() && isOK)
        {
            gridmap_pub.publish(grid_map_msg);
            r.sleep();
            ros::spinOnce();
        }
    }

    ros::shutdown();
    return 0;
}

void trackedMapPointCallback(const sensor_msgs::PointCloud2& msg)
{
    tracked_map_point = msg;
}

void mapPointCallback(const sensor_msgs::PointCloud2& msg)
{
    map_point = msg;
}

void cameraPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    camera_pose = msg;
}

void allPosePointCallback(const geometry_msgs::PoseArray& msg)
{
    all_pose_point = msg;
}

void updateGridMap(const geometry_msgs::PoseStamped& camera_pose, const sensor_msgs::PointCloud2& tracked_map_point)
{
    float pos_x = camera_pose.pose.position.x*scale_factor;
    float pos_y = camera_pose.pose.position.y*scale_factor;

    float pos_grid_x = int(floor((pos_x - grid_min_x)*norm_factor_x));
    float pos_grid_y = int(floor((pos_y - grid_min_y)*norm_factor_y));

    if(pos_grid_x < 0 || pos_grid_x >= grid_max_x - grid_min_x) return;
    if(pos_grid_y < 0 || pos_grid_y >= grid_max_y - grid_min_y) return;

    unsigned int n_pts = tracked_map_point.width;
    // std::cout << n_pts << std::endl;
    // sensor_msgs::PointCloud new_map_point;

    // Read pointcloud data
    std::vector<geometry_msgs::Pose> vec_data;
    int x_idx = sensor_msgs::getPointCloud2FieldIndex(tracked_map_point, "x");
    int y_idx = sensor_msgs::getPointCloud2FieldIndex(tracked_map_point, "y");
    int z_idx = sensor_msgs::getPointCloud2FieldIndex(tracked_map_point, "z");
    if (x_idx == -1 || y_idx == -1 || z_idx == -1) return;

    int x_offset = tracked_map_point.fields[x_idx].offset;
    int y_offset = tracked_map_point.fields[y_idx].offset;
    int z_offset = tracked_map_point.fields[z_idx].offset;

    int x_datatype = tracked_map_point.fields[x_idx].datatype;
    int y_datatype = tracked_map_point.fields[y_idx].datatype;
    int z_datatype = tracked_map_point.fields[z_idx].datatype;


    for(unsigned int i = 0; i < n_pts; i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = sensor_msgs::readPointCloud2BufferValue<float>(&tracked_map_point.data[i*tracked_map_point.point_step + x_offset], x_datatype);
        pose.position.y = sensor_msgs::readPointCloud2BufferValue<float>(&tracked_map_point.data[i*tracked_map_point.point_step + y_offset], y_datatype);
        pose.position.z = sensor_msgs::readPointCloud2BufferValue<float>(&tracked_map_point.data[i*tracked_map_point.point_step + z_offset], z_datatype);
        vec_data.push_back(pose);
    }
    
    processMapPts(vec_data, n_pts, 1, pos_grid_x, pos_grid_y);
    getGridMap();
    // showGridMap();
    // std::cout << "isOK" << std::endl;
    
}

void resetGridMap(const geometry_msgs::PoseArray& all_pose_point)
{
    if(all_pose_point.header.frame_id == "") return;

    global_visit_counter.setTo(0);
	global_occupied_counter.setTo(0);

    unsigned int n_kf = all_pose_point.poses[0].position.x;
    if ((unsigned int) (all_pose_point.poses[0].position.y) != n_kf ||
		(unsigned int) (all_pose_point.poses[0].position.z) != n_kf)
    {
		std::cout << "resetGridMap :: Unexpected formatting in the keyframe count element\n";
		return;
	}
	std::cout << "Resetting grid map with " << n_kf << " key frames\n";
    // if(n_kf % 2 != 1) return;

#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    unsigned int id = 0;
    for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id)
    {
        const geometry_msgs::Point &kf_location = all_pose_point.poses[++id].position;

        unsigned int n_pts = all_pose_point.poses[++id].position.x;
        if ((unsigned int)(all_pose_point.poses[id].position.y) != n_pts ||
			(unsigned int)(all_pose_point.poses[id].position.z) != n_pts)
        {
			std::cout << "resetGridMap :: Unexpected formatting in the point count element.\n";
			return;
		}
        float kf_pos_x = kf_location.x*scale_factor;
		float kf_pos_y = kf_location.y*scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
		int kf_pos_grid_y = int(floor((kf_pos_y - grid_min_y) * norm_factor_y));

        if (kf_pos_grid_x < 0 || kf_pos_grid_x >= int(grid_max_x - grid_min_x))
			continue;

		if (kf_pos_grid_y < 0 || kf_pos_grid_y >= int(grid_max_y - grid_min_y))
			continue;

		if (id + n_pts >= all_pose_point.poses.size()) {
			std::cout << "resetGridMap :: Unexpected end of the input array.\n";
			return;
		}
		processMapPts(all_pose_point.poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_y);
		id += n_pts;
    }
    getGridMap();
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
	double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	std::cout << "Done. Time taken: " << ttrack << " secs\n";
}

void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
	unsigned int start_id, int pos_grid_x, int pos_grid_y)
{
    unsigned int end_id = start_id + n_pts;
    for(unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
    {
        // if(pts[pt_id].position.z >= thresh_min_z && pts[pt_id].position.z < thresh_max_z)
        {
            processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
                        local_map_pt_mask, pos_grid_x, pos_grid_y);
        }
    }
}

void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied, 
	            cv::Mat &visited, cv::Mat &pt_mask, int pos_grid_x, int pos_grid_y)
{
    float pt_pos_x = curr_pt.x*scale_factor;
	float pt_pos_y = curr_pt.y*scale_factor;

    int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
	int pt_pos_grid_y = int(floor((pt_pos_y - grid_min_y) * norm_factor_y));

    if (pt_pos_grid_x < 0 || pt_pos_grid_x >= grid_max_x - grid_min_x) return;
	if (pt_pos_grid_y < 0 || pt_pos_grid_y >= grid_max_y - grid_min_y) return;

    // Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(pt_pos_grid_y, pt_pos_grid_x);
	pt_mask.at<uchar>(pt_pos_grid_y, pt_pos_grid_x) = 255;

    // Get all grid cell that the line between keyframe and map point pass through
	int x0 = pos_grid_x;
	int y0 = pos_grid_y;
	int x1 = pt_pos_grid_x;
	int y1 = pt_pos_grid_y;

    bool steep = (abs(y1 - y0) > abs(x1 - x0));
	if (steep)
    {
		std::swap(x0, y0);
		std::swap(x1, y1);
	}
	if (x0 > x1)
    {
		std::swap(x0, x1);
		std::swap(y0, y1);
	}

    int dx = x1 - x0;
	int dy = abs(y1 - y0);
	double error = 0;
	double deltaerr = ((double)dy) / ((double)dx);
	int y = y0;
	int ystep = (y0 < y1) ? 1 : -1;
	for (int x = x0; x <= x1; ++x)
    {
		if (steep)
        {
			++visited.at<int>(x, y);
		}
		else
        {
			++visited.at<int>(y, x);
		}
		error = error + deltaerr;
		if (error >= 0.5)
        {
			y = y + ystep;
			error = error - 1.0;
		}
	}
}

void getGridMap()
{
    for (int row = 0; row < int(grid_max_y - grid_min_y); ++row)
    {
		for (int col = 0; col < int(grid_max_x - grid_min_x); ++col)
        {
			int visits = global_visit_counter.at<int>(row, col);
			int occupieds = global_occupied_counter.at<int>(row, col);

            if (visits <= 0){
				grid_map.at<float>(row, col) = 0.5;
			}
			else {
				grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
			}

			if (grid_map.at<float>(row, col) >= free_thresh)
            {
                // Free
				grid_map_int.at<uchar>(row, col) = 254;
                grid_map_msg.data[row*grid_map_msg.info.width+col] = 1;
			}
			else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh)
            {
                // Unknown
				grid_map_int.at<uchar>(row, col) = 205;
                grid_map_msg.data[row*grid_map_msg.info.width+col] = -1;
			}
			else
            {
                // Obstacle
				grid_map_int.at<uchar>(row, col) = 0;
                grid_map_msg.data[row*grid_map_msg.info.width+col] = 100;
			}
			// grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
        }
    }
}

void showGridMap()
{
    // cv::imshow("grid_map_msg", cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data())));
	cv::imshow("grid_map_int", grid_map_int);
    int key = cv::waitKey(1);
    if (key == 27)
    {
		cv::destroyAllWindows();
		ros::shutdown();
		exit(0);
	}
}

bool mapGenerator(const nav_msgs::OccupancyGrid &map_data, const std::string map_name)
{
    /*
    resolution: 0.100000
    origin: [0.000000, 0.000000, 0.000000]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
    */
    int threshold_occupied = 65;
    int threshold_free = 25;
    std::string package = "orb_slam3_ros";
    std::string map_path = ros::package::getPath(package) + "/data/"+ map_name +".pgm";
    std::cout << "Writing map occupancy data to " << map_path << std::endl ;

    FILE* out = fopen(map_path.c_str(), "w");
    if (!out)
    {
        std::cout << "Couldn't save map file to " << map_path << std::endl;
        return false;
    }

    fprintf(out, "P5\n# CREATOR: RIBOT %.3f m/pix\n%d %d\n255\n",
            map_data.info.resolution, map_data.info.width, map_data.info.height);

    for(unsigned int y = 0; y < map_data.info.height; y++)
    {
        for(unsigned int x = 0; x < map_data.info.width; x++)
        {
            unsigned int i = x + (map_data.info.height - y - 1) * map_data.info.width;
            if (map_data.data[i] >= 0 && map_data.data[i] <= threshold_free)
            { // [0,free)
                fputc(254, out);
            }
            else if (map_data.data[i] >= threshold_occupied)
            { // (occ,255]
                fputc(000, out);
            }
            else { //occ [0.25,0.65]
                fputc(205, out);
            }
        }
    }

    fclose(out);

    std::string map_metadata_path = ros::package::getPath(package) + "/data/"+ map_name + ".yaml";
    std::cout << "Writing map occupancy data to " << map_metadata_path << std::endl;
    FILE* yaml = fopen(map_metadata_path.c_str(), "w");

    geometry_msgs::Quaternion orientation = map_data.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                (map_name+".pgm").c_str(),
                map_data.info.resolution,
                map_data.info.origin.position.x,
                map_data.info.origin.position.y, yaw);

    fclose(yaml);
    std::cout << "Done!." << std::endl;
    return true;
}

bool loadMap(std::string map_name, nav_msgs::OccupancyGrid& msg)
{
    std::cout << "Load 2d grid map" << std::endl;
    std::string package = "orb_slam3_ros";
    std::string map_path = ros::package::getPath(package) + "/data/"+ map_name +".pgm";
    cv::Mat map = cv::imread(map_path,0);

    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    msg.info.map_load_time = ros::Time::now();
    msg.info.width = int(grid_max_x - grid_min_x);
    msg.info.height = int(grid_max_y - grid_min_y);
    msg.info.resolution = 1.0/scale_factor;
    msg.info.origin.position.x = -(cloud_max_x - cloud_min_x)/2;
    msg.info.origin.position.y = -(cloud_max_y - cloud_min_y)/2;
    msg.info.origin.orientation.w = 1.0;

    for(int j = msg.info.height-1; j >=0; j--)
    {
        for(int i = 0; i < msg.info.width; i++)
        {
            if(int(map.at<uchar>(j,i)) == 254) msg.data.push_back(1);         // Free
            else if(int(map.at<uchar>(j,i)) == 205) msg.data.push_back(-1);   // Unknow
            else if(int(map.at<uchar>(j,i)) == 0) msg.data.push_back(100);    // Obstacle
        }
    }

    if(msg.data.size() != msg.info.height*msg.info.width) return false;
    else return true;
}