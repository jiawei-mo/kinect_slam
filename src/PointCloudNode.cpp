#include "PointCloudNode.hpp"


PointCloudNode::PointCloudNode():
	pioneer_sub(nh, "/pose1", 1),
	dep_sub(nh, "/camera/depth/image_rect", 1),
	pioneer_sync(PioneerPolicy(10), pioneer_sub, dep_sub)
{
	// state mean
	state_mean = Eigen::Vector3d::Zero();
	init_pose = Eigen::Vector3d::Zero();
	// number of images received from kinect
	num_frames = 0;
	// initialize point cloud
	cloud = PointCloudPtr(new PointCloud());
	// sync and call callback
	//pioneer_sub.registerCallback(&PointCloudNode::pose_callback, this);
	//dep_sub.registerCallback(&PointCloudNode::image_callback, this);
	pioneer_sync.registerCallback(boost::bind(&PointCloudNode::pioneer_callback, this, _1, _2));
	pcl_pub = nh.advertise<PointCloud>("pcl_map", 1);
}

void PointCloudNode::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg){
	std::cout << "pose: " << msg->header.stamp << endl;
}

void PointCloudNode::image_callback(const sensor_msgs::ImageConstPtr& msg){
	std::cout << "image: " << msg->header.stamp << endl;
}


void PointCloudNode::pioneer_callback(const geometry_msgs::PoseStampedConstPtr& state_msg, const sensor_msgs::ImageConstPtr& dep)
{
	/*
	// PROCESS STATE ESTIAMATE MESSAGE
	float x = state_msg->pose.position.x;
	float y = state_msg->pose.position.y;

	// convert from quaternions to radians
	double temp_theta_z = state_msg->pose.orientation.z;
	double temp_theta_w = state_msg->pose.orientation.w;
	double th = 2.0 * atan2(temp_theta_z, temp_theta_w);
	std::cout << "Raw state. " << ". X: " << x << ", Y: " << y << " Th: " << th << std::endl;

	state_mean << (double)x,
				  (double)y,
				  th;
	if (init_pose.isZero(0) && init_pose.isZero(1) && init_pose.isZero(2)) {
		std::cout << "Setting init pose:" << endl;
		init_pose = state_mean;
		std::cout << init_pose << endl;
	}
	*/


	// PROCESS IMAGE MESSAGES
	cv::Mat image_depth = cv_bridge::toCvCopy(dep)->image;
	// set any nan values to zero
	cv::patchNaNs(image_depth, 0.0);
	if (cv::sum(image_depth)[0] <= 0.0) {
		return;
	}

	// produce a point cloud
	PointCloudPtr pointcloud_msg (new PointCloud);
	Point pt;
	for (int y = 0; y < image_depth.rows; y+=4) {
		for (int x = 0; x < image_depth.cols; x+=4) {
			float depth = image_depth.at<short int>(cv::Point(x,y)) / 1000.0;
			if (depth > 0.0) {
				pt.x = x;
				pt.y = y;
				pt.z = depth;
				pointcloud_msg->points.push_back(pt);
			}
		}
	}
	pointcloud_msg->height = 1;
	pointcloud_msg->width = pointcloud_msg->points.size();
	std::cout << "size of new point cloud " << pointcloud_msg->points.size() << endl;

	/*
	// pass new point cloud on for further processing
	if (pointcloud_msg->points.size() > 0) {
		//cloud_append(pointcloud_msg);
		//PointCloudPtr tmp0 = remove_floor(cloud);
		//cloud = tmp0;
		//voxel_filter(0.1);
		if (num_frames % 25 == 0){
			//build_octomap();
			//publish_pointcloud();
		}
	}
	*/

}



PointCloudPtr PointCloudNode::transform_cloud(PointCloudPtr in_cloud, float dx=0.0, float dy=0.0, float dz=0.0, float theta=0.0, const std::string axis="z")
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


PointCloudPtr PointCloudNode::to_global(PointCloudPtr in_cloud)
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


void PointCloudNode::cloud_append(PointCloudPtr new_cloud)
{
	// remove some of the new points that are very far away, or super close
	PointCloudPtr no_z_outlier (new PointCloud());
	no_z_outlier = pt_filter(new_cloud, "z", 0.2, 10.0);

	// remove floor points
	PointCloudPtr only_walls (new PointCloud());
	only_walls = remove_floor(no_z_outlier);

	// Create the filtering object
	PointCloudPtr cloud_filtered (new PointCloud());
	pcl::StatisticalOutlierRemoval<Point> sor;
	sor.setInputCloud(only_walls);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	// put in global frame
	PointCloudPtr cloud_global (new PointCloud());
	cloud_global = to_global(cloud_filtered);

	// once in global each new point add to cloud
	size_t num_points = cloud_global->points.size();
	for (size_t i = 0; i < num_points; ++i) {
		Point pt;
		pt.x = cloud_global->points[i].x;
		pt.y = cloud_global->points[i].y;
		pt.z = cloud_global->points[i].z;
		//pt.rgb = cloud_global->points[i].rgb;
		cloud->points.push_back(pt);
	}
	num_frames++;

}


PointCloudPtr PointCloudNode::remove_floor(PointCloudPtr in_cloud)
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
	floor_filter.setFilterLimits(min_y + .1, min_y + 2.0);
	//floor_filter.setFilterLimitsNegative(true);
	floor_filter.filter(*no_floor_cloud);
	return no_floor_cloud;
}



void PointCloudNode::build_octomap()
{
	octomap::ColorOcTree tree(0.1);

	//octomap::Pointcloud oct_pc;
	//octomap::point3d sensor_origin(0.0f,0.0f,0.0f);

	for(PointCloud::iterator it = cloud->begin(); it != cloud->end(); it++) {
		float x=it->x, y=it->y, z=it->z;
		//uint8_t r=it->r, g=it->g, b=it->b;
		tree.updateNode((double)x, (double)y, (double)z, true);
		//tree.integrateNodeColor(x, y, z, r, g, b);
	}
	//tree.insertPointCloud(oct_pc, sensor_origin, -1, false, false);

    tree.write("my_octree.ot");
    //tree.writeBinary("my_octree.bt");
    //octomath::Pose6D origin(0,0,0,0,0,0);
    //octomap::MapNode<octomap::octree> octomap(tree,origin);
	//octomap.writeMap(newfileName);
}


PointCloudPtr PointCloudNode::pt_filter(PointCloudPtr in_cloud, const std::string field, const float min_range = 0.0, const float max_range = 1.0)
{
	// pass through filter, remove points outside of "field" range limits
	PointCloudPtr rval (new PointCloud());
	pcl::PassThrough<Point> pass; // Create the filtering object
	pass.setInputCloud(boost::make_shared<PointCloud>(*in_cloud));
	pass.setFilterFieldName(field);
	pass.setFilterLimits(min_range, max_range);
	pass.filter(*rval);
	return rval;
}

void PointCloudNode::voxel_filter(float leafsize=0.1)
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

/*
PointCloudPtr PointCloudNode::icp(PointCloudPtr cloud)
{

	// ICP alginment of clouds
	pcl::IterativeClosestPoint<Point, Point> icp;
	icp.setInputSource(cloud);
	icp.setInputTarget(new_cloud);
	PointCloudPtr tmp (new PointCloud());
	icp.align(*tmp);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	cloud = tmp;
}
*/



PointCloudPtr PointCloudNode::simulate_circle(int num_points, float num_loops) {
	PointCloudPtr rval (new PointCloud());
	//int rgb1 = 255 << 16 | 0 << 8 | 0;
	for (size_t i = 0; i < num_points; ++i) {
		Point pt;
		float pos = PI * 2.0f * num_loops * ((1.0f * i) / (1.0f * num_points));
		pt.x = cos(pos);
		pt.y = sin(pos);
		pt.z = 0.0;
		//pt.rgb=rgb1;
		rval->points.push_back(pt);
	}
	rval->width = (int) rval->points.size ();
  	rval->height = 1;
	rval->points.resize (rval->width * rval->height);
	return rval;
}



PointCloudPtr PointCloudNode::simulate_square(int num_points, float num_loops) {
	PointCloudPtr rval (new PointCloud());
	//int rgb1 = 255 << 16 | 0 << 8 | 0;
	int side_len = static_cast<int> (floor( (float)num_points / 4.0f ));

	// base and top of square
	for (size_t i = 0; i < side_len; ++i) {
		// bottom
		Point bottom;
		bottom.x = 1.0f + (1.0f * i / (float)side_len);
		bottom.y = 1.0;
		bottom.z = 1.0;
		//bottom.rgb=rgb1;
		rval->points.push_back(bottom);
		// top
		Point top;
		top.x = 1.0f + (1.0f * i / (float)side_len);
		top.y = 2.0;
		top.z = 1.0;
		//top.rgb=rgb1;
		rval->points.push_back(top);
		// left
		Point left;
		bottom.x = 1.0;
		bottom.y = 1.0f + (1.0f * i / (float)side_len);
		bottom.z = 1.0;
		//bottom.rgb=rgb1;
		rval->points.push_back(bottom);
		// right
		Point right;
		right.x = 2.0;
		right.y = 1.0 + (1.0f * i / (float)side_len);
		right.z = 1.0;
		//right.rgb=rgb1;
		rval->points.push_back(right);
	}
	rval->width = (int) rval->points.size ();
  	rval->height = 1;
	rval->points.resize (rval->width * rval->height);
	return rval;
}



void PointCloudNode::print_cloud(PointCloudPtr in_cloud)
{
	cloud_sz = in_cloud->points.size();
	for (size_t i = 0; i < cloud_sz; ++i) {
			std::cout << "    " <<
			in_cloud->points[i].x << " " <<
			in_cloud->points[i].y << " " <<
			in_cloud->points[i].z << std::endl;
	}
}

void PointCloudNode::visualize_cloud(PointCloudPtr in_cloud)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	//pcl::visualization::PointCloudColorHandlerRGBField<Point> rgbA(in_cloud);
	viewer.addPointCloud(in_cloud, "reference_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reference_cloud");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

void PointCloudNode::publish_pointcloud()
{
	cloud->header.frame_id = "/map";
	pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
	pcl_pub.publish(cloud);
}
