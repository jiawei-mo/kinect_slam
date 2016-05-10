#include "MapServerNode.hpp"
#include <math.h>
MapServerNode::MapServerNode(): 
pcl_sub(nh, "/pcl_current_frame", 1),
pose_sub(nh, "/pose1", 1),
sync(MapSyncPolicy(10), pcl_sub, pose_sub)
{
	sync.registerCallback(boost::bind(&MapServerNode::pclMessageCallback, this, _1, _2));
	map_pub = nh.advertise<PointCloud>("pcl_map", 50);
}

void MapServerNode::pclMessageCallback(const PointCloud::ConstPtr& pcl_pts, const geometry_msgs::PoseStampedConstPtr& pose)
{
	// std::cout<<"pose1";
	double r_x = pose->pose.position.x;
	double r_y = pose->pose.position.y;
	double ro_z = pose->pose.orientation.z;
	double ro_w = pose->pose.orientation.w;
	double r_theta = 2.0 * atan2(ro_z, ro_w);
	for(int i=0; i<pcl_pts->width; i++)
	{
	// std::cout<<"pose2";

		double p_x = pcl_pts->points[i].x;
		double p_y = pcl_pts->points[i].y;
		double p_z = pcl_pts->points[i].z;
		std::vector<double> n_p;
		n_p.push_back(r_x + p_x*cos(r_theta) - p_y*sin(r_theta));
		n_p.push_back(r_y + p_x*sin(r_theta) + p_y*cos(r_theta));
		n_p.push_back(p_z);
		map.push_back(n_p);
	}

	PointCloud::Ptr msg (new PointCloud);
  	msg->header.frame_id = "/map";
  	msg->height = 1;
  	msg->width = map.size();
  	msg->points.resize(msg->height * msg->width);
  	msg->is_dense = false;
  	for(int i=0; i<map.size(); i++)
  	{
	// std::cout<<"pose3";

	    msg->points[i].x = map[i][0];
	    msg->points[i].y = map[i][1];
	    msg->points[i].z = map[i][2];
  	}
	pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
	map_pub.publish(msg);
	// std::cout<<"pose4";

}