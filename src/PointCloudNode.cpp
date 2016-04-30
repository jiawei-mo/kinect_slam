#include "PointCloudNode.hpp"


PointCloudNode::PointCloudNode():
	//img_sub(nh, "/camera/depth_registered/points", 1), // from pioneer
	img_sub(nh, "/camera/rgb/image_color", 1), // for simulation
	dep_sub(nh, "/camera/depth/image", 1),
	//info_sub(nh, "/camera/rgb/camera_info", 1),
	info_sub(nh, "/camera/depth/camera_info", 1),
	sync(KinectSyncPolicy(10), img_sub, dep_sub, info_sub)
{
	//sync.registerCallback(boost::bind(&PointCloudNode::pcl_callback, this, _1, _2, _3));
	sync.registerCallback(boost::abind(&PointCloudNode::sim_pcl_callback, this, _1, _2, _3));
	// state mean
	state_mean = Eigen::Vector3d::Zero();
	init_pose = Eigen::Vector3d::Zero();
	// point cloud size
	cloud_sz = 0;
	// number of images received from kinect
	num_frames = 0;
	// initialize point cloud
	cloud = PointCloudPtr(new PointCloud());

	// subscribe to the robot state data stream
	//state_sub = nh.subscribe("pose", 1, &PointCloudNode::state_callback, this);
	state_sub = nh.subscribe("pose", 1, &PointCloudNode::sim_state_callback, this);
}


void PointCloudNode::sim_pcl_callback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
{
	cv::Mat image_color = cv_bridge::toCvCopy(img)->image;
	cv::Mat image_depth = cv_bridge::toCvCopy(dep)->image;
	cv::patchNaNs(image_depth, 0.0);

	// verify the color looks legitimate
	/*
	try {
		cv::imshow("color", image_color);
		cv::waitKey(1);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
	}
	*/

	// verify the depth looks legitimate
	/*
	try {
		double min1, max1;
		cv::minMaxIdx(image_depth, &min1, &max1);
		cv::imshow("depth", image_depth/max1);
		cv::waitKey(1);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
	}
	*/


	// get camera intrinsics
	float fx = info->K[0];
	float fy = info->K[4];
	float cx = info->K[2];
	float cy = info->K[5];

	// produce a point cloud
	PointCloudPtr pointcloud_msg (new PointCloud);
	//pointcloud_msg->header = dep->header;
	Point pt;
	for (int y = 0; y < image_color.rows; y+=5) {
		for (int x = 0; x < image_color.cols; x+=5) {
			float depth = image_depth.at<short int>(cv::Point(x,y)) / 1000.0;
			cv::Vec3b color = image_color.at<cv::Vec3b>(cv::Point(x,y));
			if (depth > 0.0) {
				pt.x = (x - cx) * depth / fx;
				pt.y = (y - cy) * depth / fy;
				pt.z = depth;
				pt.rgb = (int)color[0] << 16 | (int)color[1] << 8 | (int)color[2];
				pointcloud_msg->points.push_back(pt);
			}
		}
	}
	pointcloud_msg->height = 1;
	pointcloud_msg->width = pointcloud_msg->points.size();
	num_frames++;
	cloud_append(pointcloud_msg);
	//voxel_filter(cloud, 1.0);
	build_octomap();

}


void PointCloudNode::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg, const sensor_msgs::ImageConstPtr& dep, const sensor_msgs::CameraInfoConstPtr& info)
{
	// add points in message to the point cloud attribute
	// first convert to pcl object
	pcl::PCLPointCloud2::Ptr pcl_obj;
	pcl_conversions::toPCL(*pcl_msg, *pcl_obj);
	// then to a point cloud
	PointCloudPtr new_cloud (new PointCloud());
	pcl::fromPCLPointCloud2(*pcl_obj, *new_cloud);

	// add to data
	cloud_append(new_cloud);

	// filter the cloud to remove overly dense regions of points
	voxel_filter(cloud, 0.1);

	// convert cloud to PointCloud2
	//pcl::PCLPointCloud2::Ptr cloud_msg (new pcl::PCLPointCloud2 ());
	//toPCLPointCloud2(*cloud, *cloud_msg);
	// publish to make available to octomap
	//pcl_pub.publish(cloud_msg);

	// just call conversion to octomap
	build_octomap();
}


void PointCloudNode::sim_state_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// read in the pose information and save it in state_mean
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;

	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
	double th = tf::getYaw(pose.getRotation());
	//float th = msg->pose.pose.orientation.z;

	state_mean << x,
				  y,
				  (float)th;
	if (init_pose.isZero(0) && init_pose.isZero(1) && init_pose.isZero(2)) {
		std::cout << "Setting init pose:" << endl;
		init_pose = state_mean;
		std::cout << init_pose << endl;
	}
	std::cout << "X: " << x << ", Y: " << y << " Th: " << th << std::endl;
}

void PointCloudNode::state_callback(const geometry_msgs::Pose2D& pose_msg)
{
	// read in the pose information and save it in state_mean
	float x = pose_msg.x;
	float y = pose_msg.y;
	float th = pose_msg.theta;

	state_mean << x,
				  y,
				  th;
}


void PointCloudNode::build_octomap()
{
	octomap::ColorOcTree tree(0.25);

	//octomap::Pointcloud oct_pc;
	//octomap::point3d sensor_origin(0.0f,0.0f,0.0f);

	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		float x=it->x, y=it->y, z=it->z;
		uint8_t r=it->r, g=it->g, b=it->b;
		tree.updateNode((double)x, (double)y, (double)z, true);
		tree.integrateNodeColor(x, y, z, r, g, b);
	}
	//tree.insertPointCloud(oct_pc, sensor_origin, -1, false, false);

    tree.write("my_octree.ot");
    //tree.writeBinary("my_octree.bt");
    //octomath::Pose6D origin(0,0,0,0,0,0);
    //octomap::MapNode<octomap::octree> octomap(tree,origin);
	//octomap.writeMap(newfileName);
}


PointCloudPtr PointCloudNode::to_global(PointCloudPtr new_cloud)
{
	// this just implements a specific transformation using transform_cloud
	// point cloud should already be aligned to robot frame

	/*
	// ROBOTS XYZ may be different than image XYZ!!!!
	// rotate about robot z then robot x
	float img_dx = state_mean(1) - init_pose(1);
	float img_dy = 0.0;
	float img_dz = state_mean(0) - init_pose(0);
	float img_dth = state_mean(2) - init_pose(2);
	PointCloudPtr new_global = transform_cloud(new_cloud, img_dx, img_dy, img_dz, img_dth, "z");
	*/

	float dx = state_mean(0) - init_pose(0);
	float dy = state_mean(1) - init_pose(1);
	float theta = state_mean(2) - init_pose(2);
	PointCloudPtr new_global = transform_cloud(new_cloud, dx, dy, 0.0, theta, "z");

	return new_global;
}

void PointCloudNode::cloud_append(PointCloudPtr new_cloud)
{
	// remove some of the new points that are very far away, or super close
	//PointCloudPtr new_filtered = pt_filter(new_cloud, "z", 0.1, 3.0); // keep pts with depth 0.1m to 3m

	// Create the filtering object
	PointCloudPtr cloud_filtered (new PointCloud());
	pcl::StatisticalOutlierRemoval<Point> sor;
	sor.setInputCloud(new_cloud);
	sor.setMeanK (25);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	// first orient the new cloud so that it's in the robot frame
	// i.e. the image-z (depth) corresponds to robot-x
	// the image-y (rows) corresponds to robots-z
	// the image x (cols) corresponds to robots-y
	PointCloudPtr tmp = transform_cloud(cloud_filtered, 0.0,0.0,0.0, PI/2.0, "y");
	PointCloudPtr new_robot = transform_cloud(tmp, 0.0,0.0,0.0, PI/2.0, "x");


	// next, transform into global frame based on new robot state
	PointCloudPtr new_global = to_global(new_robot);

	/*
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
	num_frames++;
	*/

	// once in global each new point add to cloud
	size_t num_points = new_global->points.size();
	for (size_t i = 0; i < num_points; ++i) {
		Point pt;
		pt.x = new_global->points[i].x;
		pt.y = new_global->points[i].y;
		pt.z = new_global->points[i].z;

		// save to PointCloudNode attribute
		cloud->points.push_back(pt);
		cloud_sz++;
	}
	num_frames++;

}



PointCloudPtr PointCloudNode::simulate_circle(int num_points, float num_loops) {
	PointCloudPtr rval (new pcl::PointCloud<pcl::PointXYZRGB>());
	int rgb1 = 255 << 16 | 0 << 8 | 0;
	for (size_t i = 0; i < num_points; ++i) {
		pcl::PointXYZRGB pt;
		float pos = PI * 2.0f * num_loops * ((1.0f * i) / (1.0f * num_points));
		pt.x = cos(pos);
		pt.y = sin(pos);
		pt.z = 0.0;
		pt.rgb=rgb1;
		rval->points.push_back(pt);
	}
	rval->width = (int) rval->points.size ();
  	rval->height = 1;
	rval->points.resize (rval->width * rval->height);
	return rval;
}



PointCloudPtr PointCloudNode::simulate_square(int num_points, float num_loops) {
	PointCloudPtr rval (new pcl::PointCloud<pcl::PointXYZRGB>());
	int rgb1 = 255 << 16 | 0 << 8 | 0;
	int side_len = static_cast<int> (floor( (float)num_points / 4.0f ));

	// base and top of square
	for (size_t i = 0; i < side_len; ++i) {
		// bottom
		pcl::PointXYZRGB bottom;
		bottom.x = 1.0f + (1.0f * i / (float)side_len);
		bottom.y = 1.0;
		bottom.z = 1.0;
		bottom.rgb=rgb1;
		rval->points.push_back(bottom);
		// top
		pcl::PointXYZRGB top;
		top.x = 1.0f + (1.0f * i / (float)side_len);
		top.y = 2.0;
		top.z = 1.0;
		top.rgb=rgb1;
		rval->points.push_back(top);
		// left
		pcl::PointXYZRGB left;
		bottom.x = 1.0;
		bottom.y = 1.0f + (1.0f * i / (float)side_len);
		bottom.z = 1.0;
		bottom.rgb=rgb1;
		rval->points.push_back(bottom);
		// right
		pcl::PointXYZRGB right;
		right.x = 2.0;
		right.y = 1.0 + (1.0f * i / (float)side_len);
		right.z = 1.0;
		right.rgb=rgb1;
		rval->points.push_back(right);
	}
	rval->width = (int) rval->points.size ();
  	rval->height = 1;
	rval->points.resize (rval->width * rval->height);
	return rval;
}

PointCloudPtr PointCloudNode::pt_filter(PointCloudPtr cloud, const std::string field, const float min_range = 0.0, const float max_range = 1.0)
{
	// pass through filter, remove points outside of "field" range limits
	PointCloudPtr rval (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PassThrough<pcl::PointXYZRGB> pass; // Create the filtering object
	pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
	pass.setFilterFieldName(field);
	pass.setFilterLimits(min_range, max_range);
	pass.filter(*rval);
	return rval;
}

void PointCloudNode::voxel_filter(PointCloudPtr cloud, float leafsize=0.1)
{
	// Divide space into voxels, replaces points within a voxels by their centroids.
	PointCloudPtr rval (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	// one cell within each leafsize m
	sor.setLeafSize(leafsize, leafsize, leafsize);
	// apply filtering, assign results to rval
	sor.filter(*rval);
	cloud = rval;
}

void PointCloudNode::print_cloud(PointCloudPtr cloud)
{
	size_t cloud_sz = cloud->points.size();
	for (size_t i = 0; i < cloud_sz; ++i) {
			std::cout << "    " <<
			cloud->points[i].x << " " <<
			cloud->points[i].y << " " <<
			cloud->points[i].z << std::endl;
	}
}

void PointCloudNode::visualize_cloud(PointCloudPtr cloud)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbA(cloud);
	viewer.addPointCloud(cloud, "reference_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reference_cloud");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(100);
	}
}

PointCloudPtr PointCloudNode::transform_cloud(PointCloudPtr cloud,
	float dx=0.0, float dy=0.0, float dz=0.0, float theta=0.0, const std::string axis="z")
{
	// create transformation matrix
	Eigen::Affine3f T = Eigen::Affine3f::Identity();
	T.translation() << dx, dy, dz;
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
	//std::cout << "rotation matrix: " << std::endl;
	//std::cout << T.matrix() << std::endl;
	PointCloudPtr rval (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud (*cloud, *rval, T);
	return rval;
}

