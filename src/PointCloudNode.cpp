#inlcude "PointCloudNode.hpp"



PointCloudNode::PointCloudNode():
	img_sub(nh, "/camera/depth_registered/points", 1),
	sync(KinectSyncPolicy(10), img_sub)
{
	// state mean
	state_mean = Eigen::Vector3d::Zero();
	// point cloud size
	cloud_sz = 0;
	// initialize point cloud
	cloud = PointCloudPtr(new PointCloud());
	// subscribe to the point cloud data stream
	//ros::Subscriber points_sub = nh.subscribe("/camera/depth_registered/points", 1, &PointCloudNode::pcl_callback, this);

	// subscribe to the robot state data stream
	ros::Subscriber state_sub = nh.subscribe("pose", 1, &PointCloudNode::state_callback, this);

	// octomap can be done just by publishing pointcloud2...
	//ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_pcl", 1);

	octomap::ColorOcTree tree( 0.1 );
}



void PointCloudNode::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
{
	// add points in message to the point cloud attribute
	// first convert to pcl object
	pcl::PCLPointCloud2::Ptr pcl_obj;
	pcl_conversions::toPCL(*pcl_msg, *pcl_obj);
	// then to a point cloud
	PointCloudPtr new_cloud (new PointCloud());;
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
	// convert poi

	//octomap::Pointcloud oct_pc;
	//octomap::point3d sensor_origin(0.0f,0.0f,0.0f);

	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++) {
		//octomap::point3d new_pt ((float) it->x, (float) it->y, (float) it->z);
		//oct_pc.push_back(new_pt);
		tree.updateNode(new_pt, true);
		tree.integrateNodeColor( it->x, it->y, it->z, it->r, it->g, it->b );
	}
	//tree.insertPointCloud(oct_pc, sensor_origin, -1, false, false);

    //tree.write("my_octree.ot");
    tree.writeBinary("my_octree.bt");
}


PointCloudPtr PointCloudNode::to_global(PointCloudPtr new_cloud)
{
	// this just implements a specific transformation using transform_cloud
	float dx = state_mean(0);
	float dy = state_mean(1);
	float theta = state_mean(2);
	PointCloudPtr new_global = transform_cloud(new_cloud, dx, dy, 0.0, theta);
	return new_global;
}

void PointCloudNode::cloud_append(PointCloudPtr new_cloud)
{
	// remove some of the new points that are very far away, or super close
	PointCloudPtr new_filtered = pt_filter(new_cloud, "z", 0.1, 3.0); // keep pts with depth 0.1m to 3m

	// first need to do some transformations in here involving state mean
	PointCloudPtr new_global = to_global(new_filtered);

	// once in global each new point add to cloud
	size_t num_points = new_global->points.size();
	for (size_t i = 0; i < num_points; ++i) {
		Point pt;
		pt.x = new_global->points[i].x;
		pt.y = new_global->points[i].y;
		pt.z = new_global->points[i].z;
		pt.rgb = new_global->points[i].rgb;

		// save to PointCloudNode attribute
		cloud->points.push_back(pt);
		cloud_sz++;
	}
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
	float dx=0.0, float dy=0.0, float dz=0.0, float theta=0.0)
{
	// TODO create option to specify the axis of rotation
	// create transformation matrix
	Eigen::Affine3f T = Eigen::Affine3f::Identity();
	T.translation() << dx, dy, dz;
	if (theta != 0.0) {
		T.rotate (Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	}
	PointCloudPtr rval (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud (*cloud, *rval, T);
	return rval;
}

