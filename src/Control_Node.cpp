#include "Control_Node.hpp"

Control_Node::Control_Node()
{
  double ini_CLOCK_SPEED=1;
  double Initialization_time=6;
  double ini_count=0;
  double distance_maintain =0.8;
  // ros::Rate ini_rate(ini_CLOCK_SPEED);
  turn_time=ros::Time::now();
  /*while (ros::ok() && ini_count<Initialization_time)
  {
    ini_count++;
    ini_rate.sleep();
  }
  */
  first_turn = 1;
  turn_count = 0;
  distance_maintain=0.8;
  correction_threshold=0.1;
  follow_wall_time = ros::Time::now();
  action_lock=0;
  sonar = nh.subscribe("RosAria/sonar",1, &Control_Node::sonarMeassageReceived,this);
  pose_correct=nh.subscribe("/pose1",1, &Control_Node::poseMeassageReceived,this);
  point_data=nh.subscribe("raw_depth",1,&Control_Node::pointMeassageReceived,this);
  pose_corrected = 0;
  current_theta = 0;
  turn_flag=0;
}

void Control_Node::sonarMeassageReceived(const sensor_msgs::PointCloud &msg)
{
  char action;
  current_time=ros::Time::now();
  pose_corrected = myCtrl.check_pose(current_theta);
  //std::cout<<"check_pose is " << pose_corrected<<std::endl;
  if(msg.points[0].y>=LEFT_AVAILABLE && (current_time.sec-turn_time.sec>50)&& (current_time.sec-follow_wall_time.sec>8) &&!action_lock && pose_corrected)    //30s for turn_lock
  {
    action_lock = 1; //locking the system to prevent operation conflict
    action_lock = myCtrl.turn_left();
    turn_count+=action_lock;
    turn_flag =2;
    turn_time=current_time;
    first_turn = 0;
  }
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[2].x>OBSTACLE_SIDES &&msg.points[1].x>OBSTACLE_SIDES && !action_lock) //avoid obstacle left
  {
    action_lock = 1;
    action_lock = myCtrl.turn_left();
    action_lock = 1;
    action_lock = myCtrl.turn_right();
  }
  if(msg.points[3].x<OBSTACLE_FRONT && msg.points[4].x>OBSTACLE_SIDES &&msg.points[5].x>OBSTACLE_SIDES && !action_lock) //avoid obstacle left
  {
    action_lock = 1;
    action_lock = myCtrl.turn_right();
    action_lock = 1;
    action_lock = myCtrl.turn_left();
  }
  //follow wall
  current_time = ros::Time::now();
 if ((msg.points[0].y<=0.7 || msg.points[6].y>-0.9) && !action_lock && ((current_time.sec-follow_wall_time.sec>20 && current_time.sec-turn_time.sec>20 )|| first_turn)) //20,20
  {
   follow_wall_time = ros::Time::now();
   action_lock = 1;
   if ((msg.points[0].y<=0.7) && (msg.points[6].y>-0.9))
   {

   }
   else if (msg.points[0].y<=0.7)
   {
    action_lock = myCtrl.follow_wall(1,1,msg.points[0].y); //slightly turn right
    //action_lock = 1;
    //action_lock = myCtrl.follow_wall(0,2,msg.points[0].y);
    turn_flag = 2;
   }
   else
   {
    action_lock = myCtrl.follow_wall(0,1,msg.points[6].y); //slightly turn left
    //action_lock =1;
    //action_lock = myCtrl.follow_wall(1,2,msg.points[6].y);
    turn_flag = 2;
   }
  }
 //pose correction using EKF estimation
   current_time = ros::Time::now();
   if (!action_lock)
   {
   	 if(turn_flag>0 || current_time.sec-turn_time.sec<10)
   	 {
      action_lock = 1;
      action_lock = myCtrl.pose_correction(current_EKF_theta,turn_flag);
      turn_flag--;
     }
     else
     {
      action_lock = 1;
      action_lock = myCtrl.pose_correction(current_theta,0);
     }
   }
}

void Control_Node::poseMeassageReceived(const geometry_msgs::PoseStamped &msg)
{
  double temp_theta_z = msg.pose.orientation.z;
  double temp_theta_w = msg.pose.orientation.w;
  current_EKF_theta=atan2(temp_theta_z,temp_theta_w)*2;
}


void Control_Node::pointMeassageReceived(const kinect_slam::LandmarkMsgConstPtr& msg)
{
  // unpack message into an eigen matrix
  int n_obs = msg->landmark_count;
  //std::cout << "RANSAC GIVEN " << n_obs << " POINTS" << std::endl;
  Eigen::MatrixXd mat(n_obs, 2);
  for (int i=0; i < n_obs; i++) {
    mat(i,0) = msg->position_x[i];
    mat(i,1) = msg->position_y[i];

    /*
    // DATA FOR TESTING
    if (i % 3 == 0) {
      // do very random point
      double noisy_x = (2.0 * (double)rand() / RAND_MAX) - 1.0;
      double noisy_y = (2.0 * (double)rand() / RAND_MAX) - 1.0;
      mat(i,0) = noisy_x;
      mat(i,1) = 1.0 + noisy_y;
    } else {
      // generate random data along x, with y varying by +- .25
      double noise_in_x = (0.1 * (double)rand() / RAND_MAX) - 0.5; // -0.05 to 0.05
      double noisy_y = (0.5 * (double)rand() / RAND_MAX) - 0.25; // -0.25 to 0.25
      mat(i,0) = (double)i + noise_in_x;
      mat(i,1) = 1.0 + noisy_y;
    }
   */
  }

  // Define parameters for RANSAC
  int max_iter = 500;
  double dist_thres = 0.01; // threshold of distance between points and fitting line
  int inlier_threshold = n_obs / 2;
  int early_stop_threshold = n_obs  - (n_obs / 5);

  // intialize RANSAC
  int best_inlier_ct = 0;
  double best_slope = 0.0;
  double best_intercept = 0.0;
  for (int iter = 0; iter < max_iter; iter++) {
    // select random sample of points
    int id1 = rand() % n_obs; // random int index, should be without replacement though
    int id2 = rand() % n_obs;
    Eigen::Vector2d pt1(mat(id1,0), mat(id1,1));
    Eigen::Vector2d pt2(mat(id2,0), mat(id2,1));

    // compute distance between and obervations and the random line
    Eigen::Vector2d dif = pt2 - pt1;
    Eigen::Vector2d unit_dif = dif / dif.norm();
    Eigen::Vector2d norm_vec(-1.0 * unit_dif(1), unit_dif(0));
    Eigen::VectorXd dist = (mat.rowwise() - pt1.transpose()) * norm_vec;

    // compute the inliers not exceeding the threshold
    std::vector<int> inliers;
    for (int i = 0; i < n_obs; i++) {
      if (fabs(dist(i)) <= dist_thres) {
        inliers.push_back(i);
      }
    }
    if (inliers.size() > inlier_threshold && inliers.size() > best_inlier_ct) {
      // found a good line, update model parameters
      best_inlier_ct = inliers.size();
      best_slope = (pt2(1) - pt1(1)) / (pt2(0) - pt1(0));
      best_intercept = pt1(1) - best_slope * pt1(0);

      // early stop threshold
      if (inliers.size() > early_stop_threshold && iter > max_iter / 4) {
        //std::cout << "STOPPING EARLY, ITER: " << iter << std::endl;
        break;
      }
    }

  }
  // save slope of the line
  //std::cout << "STOPING RANSAC WITH " << best_inlier_ct << " POINTS" << std::endl;
  //std::cout << "BEST SLOPE: " << best_slope << std::endl;
  //std::cout << "BEST INTERCEPT: " << best_intercept << std::endl;
  current_theta = best_slope;
}
