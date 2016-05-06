#include "MappingSimulation.hpp"


MappingSimulation::MappingSimulation():
	simulation_sub(nh, "pose", 1),
	//simulation_sub(nh, "/tf", 1),
	dep_sub(nh, "/camera/depth/image", 1), // for simulation
	info_sub(nh, "/camera/rgb/camera_info", 1),
	simulation_sync(SimulationPolicy(10), simulation_sub, dep_sub, info_sub)
{
	simulation_sync.registerCallback(boost::bind(&MappingSimulation::simulation_callback, this, _1, _2, _3));
	// state mean
	state_mean = Eigen::Vector3d::Zero();
	init_pose = Eigen::Vector3d::Zero();
	// point cloud size
	cloud_sz = 0;
	// number of images received from kinect
	num_frames = 0;
	// initialize point cloud
	cloud = PointCloudPtr(new PointCloud());
}


//void MappingSimulation::simulation_callback(const geometry_msgs::TransformStamped::ConstPtr& state_msg, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
void MappingSimulation::simulation_callback(const nav_msgs::Odometry::ConstPtr& state_msg, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
{
	// PROCESS STATE ESTIAMATE MESSAGE
	float x = state_msg->pose.pose.position.x;
	float y = state_msg->pose.pose.position.y;

	// extract orientation / yaw from pose data
	/*
	// this is still probably wrong
	tf::Pose pose;
	tf::poseMsgToTF(state_msg->pose.pose, pose);
	double th = tf::getYaw(pose.getRotation());
	float th = (float)current_theta;
	*/

	float temp_theta_z = state_msg->pose.pose.orientation.z;
	float temp_theta_w = state_msg->pose.pose.orientation.w;
	float th = 2.0 * atan2(temp_theta_z, temp_theta_w);


	// save estimates in class variables
	state_mean << x,
				  y,
				  th;
	if (init_pose.isZero(0) && init_pose.isZero(1) && init_pose.isZero(2)) {
		std::cout << "Setting init pose:" << endl;
		init_pose = state_mean;
		std::cout << init_pose << endl;
	}

	// PROCESS IMAGE MESSAGES
	//cv::Mat image_color = cv_bridge::toCvCopy(img)->image;
	cv::Mat image_depth = cv_bridge::toCvCopy(dep)->image;
	// set any nan values to zero
	cv::patchNaNs(image_depth, 0.0);

	// get camera intrinsics
	float fx = info->K[0];
	float fy = info->K[4];
	float cx = info->K[2];
	float cy = info->K[5];

	// produce a point cloud
	PointCloudPtr pointcloud_msg (new PointCloud);
	//pointcloud_msg->header = dep->header;
	Point pt;
	for (int y = 0; y < image_depth.rows; y++) {
		for (int x = 0; x < image_depth.cols; x++) {
			float depth = image_depth.at<short int>(cv::Point(x,y)) / 1000.0;
			//cv::Vec3b color = image_color.at<cv::Vec3b>(cv::Point(x,y));
			if (depth > 0.0) {
				pt.x = (x - cx) * depth / fx;
				pt.y = (y - cy) * depth / fy;
				pt.z = depth;
				//pt.rgb = (int)color[0] << 16 | (int)color[1] << 8 | (int)color[2];
				pointcloud_msg->points.push_back(pt);
			}
		}
	}
	pointcloud_msg->height = 1;
	pointcloud_msg->width = pointcloud_msg->points.size();


	// pass new point cloud on for further processing
	cloud_append(pointcloud_msg);
	if (num_frames % 10 == 0) {
		// remove floor points
		PointCloudPtr tmp0 = remove_floor(cloud);
		cloud = tmp0;
		voxel_filter(0.1);
		build_octomap();
	}

}



PointCloudPtr MappingSimulation::transform_cloud(PointCloudPtr in_cloud,
	float dx=0.0, float dy=0.0, float dz=0.0, float theta=0.0, const std::string axis="z")
{
	// create transformation matrix
	Eigen::Affine3f T = Eigen::Affine3f::Identity();
	if (theta != 0.0) {

		if (axis == "x") {
			T.rotate (Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
		} else if (axis == "y") {
			T.rotate (Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
		} else {
			// assume z
			T.rotate (Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
		}
	}
	T.translation() << dx, dy, dz;
	PointCloudPtr rval (new PointCloud());
	pcl::transformPointCloud(*in_cloud, *rval, T);
	return rval;
}


PointCloudPtr MappingSimulation::to_global(PointCloudPtr in_cloud)
{
	// this just implements a specific transformation using transform_cloud

	// first orient the new cloud so that it's aligned with robot frame
	// i.e. the image-z (depth) corresponds to robot-x
	// the image-y (rows) corresponds to robots-z
	// the image x (cols) corresponds to robots-y
	PointCloudPtr tmp1 = transform_cloud(in_cloud, 0.0,0.0,0.0, PI/2.0, "y");
	PointCloudPtr tmp2 = transform_cloud(tmp1, 0.0,0.0,0.0, PI/2.0, "x");

	// next, transform into global frame based on new robot state
	float dx = state_mean(0) - init_pose(0);
	float dy = state_mean(1) - init_pose(1);
	float theta = state_mean(2) - init_pose(2);
	std::cout << num_frames << ". X: " << dx << ", Y: " << dy << " Th: " << theta << std::endl;
	PointCloudPtr tmp3 = transform_cloud(tmp2, dx, dy, 0.0, theta, "z");

	// reorient the point cloud so the image plane is
	// parallel to robot's y-z plane
	PointCloudPtr tmp4 = transform_cloud(tmp3, 0.0,0.0,0.0, -1.0*PI/2.0, "x");
	PointCloudPtr new_global = transform_cloud(tmp4, 0.0,0.0,0.0, -1.0*PI/2.0, "y");
	return new_global;
}


void MappingSimulation::cloud_append(PointCloudPtr new_cloud)
{
	// remove some of the new points that are very far away, or super close
	PointCloudPtr filt1 = pt_filter(new_cloud, "z", 0.1, 10.0);
	//PointCloudPtr filt2 = pt_filter(filt1, "x", 1.0, 4.0);

	// Create the filtering object
	PointCloudPtr cloud_filtered (new PointCloud());
	pcl::StatisticalOutlierRemoval<Point> sor;
	sor.setInputCloud(filt1);
	sor.setMeanK (25);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	// put in global frame
	PointCloudPtr cloud_global = to_global(cloud_filtered);

	// once in global each new point add to cloud
	size_t num_points = cloud_global->points.size();
	for (size_t i = 0; i < num_points; ++i) {
		Point pt;
		pt.x = cloud_global->points[i].x;
		pt.y = cloud_global->points[i].y;
		pt.z = cloud_global->points[i].z;
		//pt.rgb = cloud_global->points[i].rgb;
		cloud->points.push_back(pt);
		cloud_sz++;
	}
	num_frames++;

}


PointCloudPtr MappingSimulation::remove_floor(PointCloudPtr in_cloud)
{
	float min_y = 99999.0;
	for(size_t i=0; i < in_cloud->points.size(); ++i) {
		if (in_cloud->points[i].y < min_y ) {
			min_y = in_cloud->points[i].y;
		}
	}
	PointCloudPtr no_floor_cloud (new PointCloud());
	pcl::PassThrough<Point> floor_filter;
	floor_filter.setInputCloud(in_cloud);
	floor_filter.setFilterFieldName("y");
	floor_filter.setFilterLimits(min_y + .1, min_y + 2.5);
	//floor_filter.setFilterLimitsNegative(true);
	floor_filter.filter(*no_floor_cloud);
	return no_floor_cloud;
}



void MappingSimulation::build_octomap()
{
	octomap::ColorOcTree tree(0.1);
	for(PointCloud::iterator it = cloud->begin(); it != cloud->end(); it++) {
		float x=it->x, y=it->y, z=it->z;
		//uint8_t r=it->r, g=it->g, b=it->b;
		tree.updateNode((double)x, (double)y, (double)z, true);
		//tree.integrateNodeColor(x, y, z, r, g, b);
	}
    tree.write("my_octree.ot");
}


PointCloudPtr MappingSimulation::pt_filter(PointCloudPtr cloud, const std::string field, const float min_range = 0.0, const float max_range = 1.0)
{
	// pass through filter, remove points outside of "field" range limits
	PointCloudPtr rval (new PointCloud());
	pcl::PassThrough<Point> pass; // Create the filtering object
	pass.setInputCloud(boost::make_shared<PointCloud>(*cloud));
	pass.setFilterFieldName(field);
	pass.setFilterLimits(min_range, max_range);
	pass.filter(*rval);
	return rval;
}

void MappingSimulation::voxel_filter(float leafsize=0.1)
{
	// Divide space into voxels, replaces points within a voxels by their centroids.
	PointCloudPtr rval (new PointCloud());
	pcl::VoxelGrid<Point> sor;
	sor.setInputCloud(cloud);
	// one cell within each leafsize m
	sor.setLeafSize(leafsize, leafsize, leafsize);
	// apply filtering, assign results to rval
	sor.filter(*rval);
	cloud = rval;
}


void MappingSimulation::print_cloud(PointCloudPtr cloud)
{
	size_t cloud_sz = cloud->points.size();
	for (size_t i = 0; i < cloud_sz; ++i) {
			std::cout << "    " <<
			cloud->points[i].x << " " <<
			cloud->points[i].y << " " <<
			cloud->points[i].z << std::endl;
	}
}

void MappingSimulation::visualize_cloud(PointCloudPtr cloud)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	//pcl::visualization::PointCloudColorHandlerRGBField<Point> rgbA(cloud);
	viewer.addPointCloud(cloud, "reference_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reference_cloud");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}


