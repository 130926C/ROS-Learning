#include "pure_persuit_local_planner.h"
#include <chrono>//std timer

namespace pure_persuit_local_planner {

static double calcDistance(geometry_msgs::PoseStamped pose1,
						   geometry_msgs::PoseStamped pose2)
{
	double x1 = pose1.pose.position.x;
	double y1 = pose1.pose.position.y;
	double x2 = pose2.pose.position.x;
	double y2 = pose2.pose.position.y;
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

PurePersuitLocalPlanner::PurePersuitLocalPlanner(){}

void PurePersuitLocalPlanner::initialize(std::string name, 
										 tf2_ros::Buffer* tf,
										 costmap_2d::Costmap2DROS* costmap_ros) 
{
	ros::NodeHandle nh("~");	
	ros::NodeHandle private_nh("~/"+name);
	private_nh.param("start_distance_err", start_distance_err_, 0.35);	//if need smoother start and rotate, change to 10.1
	private_nh.param("linear_kp", linear_kp_, 1.7);//06-04:2.0
	private_nh.param("linear_ki", linear_ki_, 0.03);
	private_nh.param("linear_kd", linear_kd_, 0.00);
	private_nh.param("rot_kp", rot_kp_, 1.0);
	private_nh.param("rot_ki", rot_ki_, 0.02);
	private_nh.param("rot_kd", rot_kd_, 0.0);
	private_nh.param("max_trans_vel", default_max_trans_vel_, 1.0);
	private_nh.param("max_rot_vel", max_rot_vel_, 1.0);
	private_nh.param("lookingforward_dist", lookingforward_dist_, 3.0);	//obstacle detect
	private_nh.param("position_precision", p_precision_, 0.01);
	private_nh.param("orientation_precision", o_precision_, 0.02);
	private_nh.param("use_map_3d", use_map_3d_, false);
	slow_down_speed = false;
  edge_state_ = 1;
	odom_sub_ = private_nh.subscribe<nav_msgs::Odometry>("/odom",1,&PurePersuitLocalPlanner::odomCB,this);
 ros::NodeHandle public_nh;
 lane_edge_sub_ = public_nh.subscribe<std_msgs::Int32>("lane_edge",1,&PurePersuitLocalPlanner::laneEdgeCB,this);
	footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("/footprint_pure_persuit" ,1  );
	lethal_points_pub_ = private_nh.advertise<visualization_msgs::Marker>("lethal_points", 100);
	carrot_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("carrot_pose",10);
	start_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("start_pose",10);
	
	pub_path_ = private_nh.advertise<nav_msgs::Path>("pure_persuit_path",1);
	if(!costmap_ros)
	{
		ROS_ERROR("costmap_ros is null!!!!!!!!!!!!!!!!!!!!!!!");
	}
	costmap_ros_ = costmap_ros;
	costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
	costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);
	tf_ = tf;
	tf_listener_ = new tf::TransformListener();
	// lookingforward_dist_ = std::min(std::min(costmap_->getSizeInMetersX(),
	// 								costmap_->getSizeInMetersY() ) - robot_inscribed_radius_/*robot radius*/,
	// 								lookingforward_dist_); 
	linear_pid1_.kp_ = linear_kp_;
	linear_pid1_.ki_ = linear_ki_;
	linear_pid1_.kd_ = linear_kd_;
	linear_pid1_.integ_limit_ = max_trans_vel_;
	linear_pid1_.output_limit_ = max_trans_vel_;

	angular_pid1_.kp_ = rot_kp_;
	angular_pid1_.ki_ = rot_ki_;
	angular_pid1_.kd_ = rot_kd_;
	angular_pid1_.integ_limit_ = max_rot_vel_;
	angular_pid1_.output_limit_ = max_rot_vel_;

	can_go_ = false;
    robot_pose_.pose.orientation.w = 1.0;
    unvalid_counter_ = 0;

	setVelocity(2);
	resetAllParam();
}
void PurePersuitLocalPlanner::laneEdgeCB(const std_msgs::Int32::ConstPtr& ptr){
	// boost::unique_lock<mutex_t> lock(access_edge_);
	edge_state_ = static_cast<int>(ptr->data);

}

void PurePersuitLocalPlanner::odomCB(const nav_msgs::Odometry::ConstPtr& ptr)
{
	boost::unique_lock<mutex_t> lock(access_v_);
	current_robot_v_ = ptr->twist.twist.linear.x;
	current_robot_w_ = ptr->twist.twist.angular.z;
	if(!use_map_3d_){
		robot_pose_.pose = ptr->pose.pose;
	}else{
		//get robot pose in map_3d coordiate
		geometry_msgs::PoseStamped robot_pose, robot_pose_w;
		robot_pose.header.stamp = ros::Time(0);
		robot_pose.header.frame_id = "base_link";
		robot_pose.pose.orientation.w = 1.0;
		try{
			tf_listener_->waitForTransform("map_3d", "base_link",ros::Time(0), ros::Duration(0.5));
			tf_listener_->transformPose("map_3d", robot_pose, robot_pose_w);
		}   
		catch (tf::TransformException &ex){
				ROS_ERROR_STREAM("Transform failure: " << ex.what());
		}
		// ROS_INFO_STREAM("ROBOT POSE in map_3d: "<< robot_pose_w);
		//get robot pose in odom
		transform_pose("odom", "map_3d", robot_pose_w.pose, robot_pose_.pose);
		// ROS_INFO_STREAM("ROBOT POSE in odometry: "<< robot_pose_);
	}

}

bool PurePersuitLocalPlanner::transform_pose(std::string parent_frame_id, 
                    						std::string child_frame_id, 
                    						const geometry_msgs::Pose& input_pose, 
                    						geometry_msgs::Pose& output_pose)
{
    geometry_msgs::PoseStamped tmp_input_pose, tmp_output_pose;
    tmp_input_pose.header.stamp = tmp_output_pose.header.stamp = ros::Time(0);
    tmp_input_pose.header.frame_id = child_frame_id;
    tmp_output_pose.header.frame_id = parent_frame_id;
    tmp_input_pose.pose = input_pose;
    // std::cout<<"child_frame_id = "<<child_frame_id<<std::endl;
    try{
        tf_listener_->waitForTransform(parent_frame_id, child_frame_id,ros::Time(0), ros::Duration(0.5));
        tf_listener_->transformPose(parent_frame_id, tmp_input_pose, tmp_output_pose);
    }   
    catch (tf::TransformException &ex){
            ROS_ERROR_STREAM("Transform cloud failure: " << ex.what());
            return false;
    }
    output_pose = tmp_output_pose.pose;
    // ROS_INFO_STREAM("output_pose: "<< tmp_output_pose);

    return true;
}

PurePersuitLocalPlanner::~PurePersuitLocalPlanner(){}

bool PurePersuitLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) 
{
	boost::unique_lock<mutex_t> lock(access_);
	orig_global_plan_.clear();
	if(orig_global_plan.size() == 0){
		ROS_ERROR("empty Global Plan!");
		return false;
	}
	orig_global_plan_ = orig_global_plan;
	can_go_ = false;
	goal_reached_ = false;
	final_goal_reached_ = false;
	resetAllParam();
	return true;
}

bool PurePersuitLocalPlanner::isGoalReached() {
	return final_goal_reached_;
}

bool PurePersuitLocalPlanner::checkGoalArrived(const std::vector<geometry_msgs::PoseStamped>& current_plan,
												double scale)
{
	double goal_x, goal_y;
	if(current_plan.empty())
	{
		ROS_ERROR("empty plan from checkGoalArrived");
		return false;
	}
	int n = current_plan.size();
	geometry_msgs::PoseStamped goal = current_plan.at(n-1);
	goal_x = goal.pose.position.x - robot_pose_.pose.position.x;
	goal_y = goal.pose.position.y - robot_pose_.pose.position.y;
	double dist_to_goal = hypot(goal_x, goal_y);
	//check if goaled
	
	if(fabs(dist_to_goal) < scale*p_precision_)//if reach the local goal
	{
		return true;
	}else{
		return false;
	}
}

bool PurePersuitLocalPlanner::checkGoalArrived(const geometry_msgs::PoseStamped& final_goal,
												double scale)
{
	double goal_x, goal_y;
	goal_x = final_goal.pose.position.x - robot_pose_.pose.position.x;
	goal_y = final_goal.pose.position.y - robot_pose_.pose.position.y;
	double dist_to_goal = hypot(goal_x, goal_y);
	//check if goaled
	
	if(fabs(dist_to_goal) < scale*p_precision_)//if reach the local goal
	{
		return true;
	}else{
		return false;
	}
}

int PurePersuitLocalPlanner::findStartPoseID(const std::vector<geometry_msgs::PoseStamped>& plan,
											int start_id)
{
	//find the robot location
	double gx, gy;
	double temp_dist = 9999.9;
	int start_idx = -1;
	double min_dist = 9999.9;
	int n = plan.size();
	if( plan.empty())
	{
		ROS_ERROR("empty plan,findStartPoseID");
		return -1;
	}
	//for(int i = 0; i < n ;i++)
	for (int i = start_id; i < n; i++) 
	{
		gx = plan.at(i).pose.position.x - robot_pose_.pose.position.x;
		gy = plan.at(i).pose.position.y - robot_pose_.pose.position.y;
		temp_dist = hypot(gx, gy);
		if (fabs(temp_dist) <= min_dist) {
			min_dist = temp_dist;
		
			start_idx = i;
		}
	}
	if(min_dist > start_distance_err_)
	{
		ROS_WARN("can not get a close start point.");
		std::cout<<"min_dist = "<<min_dist<<" start_id = "<<start_idx<<std::endl;
		start_idx = -1;	
	}
	return start_idx;
}
bool PurePersuitLocalPlanner::findLookforwardPoseIDs(int& start_id, 
													 int& lookforward_id,
													 const std::vector<geometry_msgs::PoseStamped>& plan)
{
	// advance to next goal pose, out of the p_window_ and get obstacle lookforward index value.
	// find out the pose id in the range of the radius (p_window_ and lookingforward_dist_)??why two value of radius??
	if( plan.empty())
	{
		ROS_ERROR("empty plan, findLookforwardPoseIDs");
		return false;
	}

	int n = plan.size();
	if(start_id < 0 || start_id >= n)
	{
		ROS_ERROR("unvalied start_id");
		start_id = 0;
		return false;
	}
	lookforward_id = start_id;
	int lookforward_done = 0;
	double gx,gy;
	while (lookforward_id < n) {

		if(!lookforward_done){
			gx = plan.at(lookforward_id).pose.position.x - robot_pose_.pose.position.x;
			gy = plan.at(lookforward_id).pose.position.y - robot_pose_.pose.position.y;
			double dist_lookforward = hypot(gx,gy);//distance between global plan point and robot
			if(lookforward_id < n-1 && (dist_lookforward < lookingforward_dist_)){///default is 1.5
				lookforward_id++;
			}else{
				lookforward_done = 1;
			}
		}
		if(lookforward_done ){
			return true;
		}
	}
	return false;
}

bool PurePersuitLocalPlanner::computeVelUsingPID(geometry_msgs::Twist& cmd,
										 const geometry_msgs::PoseStamped& temp_goal,
										 geometry_msgs::PoseStamped& temp_goal_pre,
										 double max_v,double max_w)//,bool force)
{
	static double v = 0;
	static double w = 0;
	
	double l = 0; 
    double delta_x = temp_goal.pose.position.x - temp_goal_pre.pose.position.x;
    double delta_y = temp_goal.pose.position.y - temp_goal_pre.pose.position.y;
		double angle =  atan2(delta_y,delta_x);
		std::cout<<"angle = "<<angle<<std::endl;

    while(angle < -M_PI)
    	angle += 2*M_PI;
    while(angle > M_PI)
    	angle -= 2*M_PI;
		double angle_input;
		if(fabs(angle) < 0.4)
			angle_input = -(angle/M_PI)*1.5;
		else 
			angle_input = -(angle/M_PI)*2;
		
		w = angular_pid1_.calc(0.0, angle_input); 
		std::cout<<"pid_w = "<<w<<std::endl;
	if (fabs(v) > max_v) {
		v = copysign(max_v, v);
	}
	if (fabs(w) > max_w) {
		w = copysign(max_w, w);
	}

	if(v < 0.0 ){
		v = 0.0 ;
		ROS_WARN("V = 0");
	}
	cmd.linear.x = v;
	if(fabs(angle) < 0.4)
		cmd.angular.z = -w ;
	else
		cmd.angular.z = -w;

	return true;
}

bool PurePersuitLocalPlanner::transformGlobalPlan(const tf::TransformListener* tf,
											      const std::vector<geometry_msgs::PoseStamped>& global_plan,
											      const std::string& global_frame,
											      std::vector<geometry_msgs::PoseStamped>& result_plan)
{
	if(global_plan.empty())
		return false;
	else if(global_plan.at(0).header.frame_id == global_frame)
	{
		result_plan = global_plan;
		return true;
	}
	//std::cout<<"transform global plan from "<<global_plan.at(0).header.frame_id<<" to "<<global_frame<<std::endl;

  	std::vector<geometry_msgs::PoseStamped> transformed_plan;
    geometry_msgs::PoseStamped plan_pose = global_plan.at(0);
		plan_pose.header.frame_id = "usbcam_link";
    try {
      // get plan_to_global_transform from plan frame to global_frame
      tf::StampedTransform plan_to_global_transform;
      tf->waitForTransform(global_frame, ros::Time(0),
                          plan_pose.header.frame_id, plan_pose.header.stamp,
                          plan_pose.header.frame_id, ros::Duration(0.01));
      tf->lookupTransform(global_frame, ros::Time(0),
                         plan_pose.header.frame_id, plan_pose.header.stamp, 
                         plan_pose.header.frame_id, plan_to_global_transform);

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;
      int i = 0;
      //now we'll transform until points are outside of our distance threshold
      while(i < (int)global_plan.size()) 
      {
        const geometry_msgs::PoseStamped& pose = global_plan.at(i);
        tf::poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);
        transformed_plan.push_back(newer_pose);
        ++i;
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan.at(0).header.frame_id.c_str());
      return false;
    }
    result_plan = transformed_plan;
    
    return true;
}

bool PurePersuitLocalPlanner::checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::PoseStamped& goal)
{
	if(plan.empty())
	{
		ROS_ERROR("empty plan, check plan");
		return false;
	}

	int start_idx = 0;
	int lookforward_idx = -1;
	if(!findLookforwardPoseIDs(start_idx, lookforward_idx, plan) || lookforward_idx == -1)
	{
		ROS_ERROR("lookforward_idx error");
		return false;
	}

std::vector<geometry_msgs::PoseStamped> path_front_robot;
geometry_msgs::PoseStamped p;
p.header.frame_id = "base_link";
p.header.stamp = ros::Time::now();
p.pose.position.y = 0;
p.pose.orientation.w = 1;
for(int i = 0; i < 9;i++)
{
	p.pose.position.x += 0.6;
	path_front_robot.push_back(p);
}

int obstacle_distance_index = 0;
	if(isTrajectoryFeasible(costmap_model_.get(),
							path_front_robot,
							start_idx,
							7, 
							obstacle_distance_index)) 
    {
			std::cout<<"start id = 0 , lookforward id = "<<std::ceil(lookforward_idx)<<std::endl;
			goal = plan[std::ceil(lookforward_idx)];
    	return true;
    }
    else 
    {//change here!!!!!!!!!!!!!
    	ROS_ERROR("obstacle on the way!");
			goal = path_front_robot[obstacle_distance_index];
			goal.pose.position.y -=5.0;
			goal.pose.position.x +=5.0;
		 	std::cout<<"obstacle_index = "<<obstacle_distance_index<<std::endl;
    	return false;
    }
}
void PurePersuitLocalPlanner::resetCarrot(	geometry_msgs::PoseStamped& pre_temp_goal,
											const std::vector<geometry_msgs::PoseStamped>& current_plan)
{
	if(current_plan.empty())
	{
		ROS_ERROR("empty plan resetCarrot");
		return;
	}
	plan_idx_ = start_idx_ + 3;
	int n = static_cast<int>(current_plan.size())-1;
	if( plan_idx_ >= n )
		plan_idx_ = n;
	reset_carrot_ = false;
	pre_temp_goal = current_plan.at(start_idx_);
	return;
}
bool PurePersuitLocalPlanner::updateCarrot(	int& id,
											geometry_msgs::PoseStamped& temp_goal,
											geometry_msgs::PoseStamped& temp_goal_pre,
											const std::vector<geometry_msgs::PoseStamped>& current_plan)
{
	static int lookforward_step = 1;
	int n = static_cast<int>(current_plan.size());

	if(current_plan.empty() || id >= n )
	{
		ROS_ERROR("empty plan or wrong id , updateCarrot");
		return false;
	}
	if( start_idx_ >= n || start_idx_ < 0)
	{
		ROS_ERROR("wrong start_idx_");
		return false;
	}
	temp_goal_pre = current_plan.at(id);//normal
	id += lookforward_step; 
	if(id > n - 1)
		id = n - 1;
	temp_goal = current_plan.at(id);
	return true;
}
void PurePersuitLocalPlanner::resetAllParam()
{
	start_idx_ = 0;
	plan_idx_ = 0;
	reset_carrot_ = true;
	goal_reached_ = false;
	final_goal_reached_ = false;
	started_ = true;
	linear_pid1_.clear();
	angular_pid1_.clear();
	unvalid_counter_ = 0;
	ROS_WARN("resetAllParam()");
}
bool PurePersuitLocalPlanner::EnsureCurrentPlan(geometry_msgs::PoseStamped& goal, geometry_msgs::PoseStamped& start)
{
	std::vector<geometry_msgs::PoseStamped> transformed_plan;
	boost::unique_lock<mutex_t> lock(access_);
	if(!transformGlobalPlan(tf_listener_, 
							orig_global_plan_, 
							costmap_ros_->getGlobalFrameID(),
							transformed_plan))
	{
 	 	ROS_ERROR("Could not transform the orig_global_plan_ to the frame of the controller, EnsureCurrentPlan");
 	 	can_go_ = false;
		unvalid_counter_++;
		start.header.stamp = ros::Time::now();
		start.header.frame_id = "base_link";
		start.pose.position.x = 0.0;
		start.pose.orientation.w = 1.0;
		goal.pose.position.x = 1.0;
		goal.pose.orientation.w = 1.0;
 	 	return false;
	}
	/* just patrol the global plan */
	if(!checkPlan(transformed_plan, goal))
	{
		can_go_ = false;
		unvalid_counter_++;
		ROS_ERROR_STREAM("unvalid global_plan, EnsureCurrentPlan"<<unvalid_counter_);
			start.header.stamp = ros::Time::now();
		start.header.frame_id = "base_link";
			start.pose.position.x = 0.0;
		start.pose.orientation.w = 1.0;
		// goal.pose.position.x = 1.0;
		// goal.pose.orientation.w = 1.0;
		return false;
	}
	if(!transformed_plan.empty()){
		start = transformed_plan[0];
		start.header.stamp = ros::Time::now();
		start.header.frame_id = "base_link";
	}
	else{
			start.header.stamp = ros::Time::now();
		start.header.frame_id = "base_link";
			start.pose.position.x = 0.0;
			start.pose.orientation.w = 1.0;
			goal.pose.position.x = 1.0;
			goal.pose.orientation.w = 1.0;
			return false;
	}
	return true;
}

bool PurePersuitLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
{
	std::cout<<"-------------------"<<std::endl;
	if(slow_down_speed){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));		
	}
	geometry_msgs::PoseStamped start_pose;
	geometry_msgs::PoseStamped temp_goal;
	EnsureCurrentPlan(temp_goal, start_pose);
	start_pub_.publish(start_pose);
	carrot_pub_.publish(temp_goal);
	double max_w, max_v;
	max_v = 1.0;
	max_w = 0.9;
	computeVelUsingPID(cmd_vel, 
						temp_goal, 
						start_pose, 
						max_v, 
						max_w);

	if(slow_down_speed){
		cmd_vel.linear.x = 0.2;
		cmd_vel.angular.z = 1.0;
	}
		else
	cmd_vel.linear.x = 1.0;
	if(edge_state_ == 0){
		cmd_vel.angular.z += 0.2;
		std::cout<<"==============1"<<std::endl;

	}else if(edge_state_ ==2){
		std::cout<<"==============2"<<std::endl;
		cmd_vel.angular.z -= 0.2;
	}

	std::cout<<"v = "<<cmd_vel.linear.x<< ", w = " <<  cmd_vel.angular.z ;
	return true;
}

bool PurePersuitLocalPlanner::slowStop(geometry_msgs::Twist& cmd_vel)
{
	double v = current_robot_v_;
	double w = current_robot_w_;
	if( (fabs(v) > 0.1) || (fabs(w) > 0.1))
	{
		cmd_vel.linear.x  = 0.5*v;
		cmd_vel.angular.z = 0.5*w;
		return false;
	}
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
	return true;
}

/**
 * @brief Check whether the planned trajectory is feasible or not.
 *
 * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
 * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
 * @param costmap_model Pointer to the costmap model
 * @param footprint_spec The specification of the footprint of the robot in world coordinates
 * @param inscribed_radius The radius of the inscribed circle of the robot
 * @param circumscribed_radius The radius of the circumscribed circle of the robot
 * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
 * @return \c true, if the robot footprint along the first part of the trajectory intersects with
 *         any obstacle in the costmap, \c false otherwise.
 */
bool PurePersuitLocalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model,
													const std::vector<geometry_msgs::PoseStamped>& path,
													int start_id,
													int look_ahead_idx, 
													int &obstacle_index) 
{
		if(path.empty())
		{
			ROS_ERROR("empty path,isTrajectoryFeasible");
			return false;
		}
		double x, y, theta;
		ros::NodeHandle nh;
		std::vector<geometry_msgs::Point> oriented_footprint;
		visualization_msgs::Marker points;
		points.header = path.at(0).header;
		points.action = visualization_msgs::Marker::ADD;
		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;
		points.scale.x = 0.05;
		points.scale.y = 0.05;
		points.color.r = 1.0f;
		points.color.b = 1.0f;
		points.color.a = 1.0f;
		if(start_id < 0 || start_id > look_ahead_idx || look_ahead_idx < 0 || look_ahead_idx >= (int)path.size())
		{
			ROS_ERROR("error in the isTrajectoryFeasible");
			return false;
		}
		for (int i = start_id; i < look_ahead_idx; i++) 
		{
			if(path.at(i).header.frame_id != costmap_ros_->getGlobalFrameID())
				getTransformed2TargetPosition(path.at(i), costmap_ros_->getGlobalFrameID(),x, y, theta);
			else 
			{
				x = path.at(i).pose.position.x;
				y = path.at(i).pose.position.y;
				theta = tf::getYaw(path.at(i).pose.orientation);
			}
			float costmap_cost = costmap_model_->footprintCost(x,y,theta,
															   costmap_ros_->getRobotFootprint());

			if (costmap_cost < 0 ){
				obstacle_index = i;
				//show the collision point 
				std::vector<geometry_msgs::Point>().swap(points.points);
				geometry_msgs::Point p;
				p.x = x;
				p.y = y;
				p.z = 0;
				points.points.push_back(p);
				lethal_points_pub_.publish(points);
				//show the collision footprint
				geometry_msgs::PolygonStamped footprint;
				footprint.header = path.at(0).header;
				costmap_2d::transformFootprint(x,y,theta,costmap_ros_->getRobotFootprint(),footprint);
				footprint_pub_.publish(footprint);
				std::cout<<"path crash"<<std::endl;
				slow_down_speed = true;
				return false;
			}
		}
		slow_down_speed = false;
		return true;
}

bool PurePersuitLocalPlanner::getTransformed2TargetPosition(const geometry_msgs::PoseStamped& pose,
															const std::string& frame_id,
															double& x, 
															double& y, 
															double& theta) 
{
	geometry_msgs::PoseStamped ps;
	if(transformPose2TargetFrame(pose,ps,frame_id))
	{
		x = ps.pose.position.x;
		y = ps.pose.position.y;
		theta = tf::getYaw(ps.pose.orientation);	
		return true;
	}else
		ROS_ERROR("ERROR IN getTransformed2mapPosition");
	return false;
}

bool PurePersuitLocalPlanner::transformPose2TargetFrame(const geometry_msgs::PoseStamped& pose,
												geometry_msgs::PoseStamped& result,
												const std::string& target_frame)
{
	tf::StampedTransform transform;
	try{
    	tf_listener_->waitForTransform(pose.header.frame_id, 
    						  target_frame,
                              ros::Time(0), 
                              ros::Duration(0.01));

	    tf_listener_->lookupTransform(target_frame, pose.header.frame_id, ros::Time(0), transform);
	    tf::Stamped<tf::Pose> tf_pose;
        tf::poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = target_frame;
        tf::poseStampedTFToMsg(tf_pose, result);
		return true;
	}
	catch (tf::TransformException &ex) {
  		ROS_ERROR("%s",ex.what());
  		ros::Duration(1.0).sleep();
  		return false;
	}																  	
}

void PurePersuitLocalPlanner::setVelocity(int level)
{
	boost::unique_lock<mutex_t> lock(access_level_);
  	std::cout<<"set speed level = "<<level<<std::endl;
    if(level == 1)//average: 0.3, max: 0.4
    {
    	linear_pid1_.ki_  = linear_ki_ * 2.7;//default_linear_ki * 16 / 6
      	linear_pid1_.integ_limit_ = default_max_trans_vel_ * 0.4;
	  	linear_pid1_.output_limit_ = default_max_trans_vel_ * 0.4;
	    angular_pid1_.kp_ = rot_kp_ - 0.8;//default = 1.8

	  	max_trans_vel_ = default_max_trans_vel_ * 0.4;
	  	std::cout<<"max v = "<<max_trans_vel_<<"k_i = "<< linear_pid1_.ki_ <<std::endl;
    }
    else if(level == 2)//average: 0.5, max: 0.6
    {
      linear_pid1_.ki_  = linear_ki_ * 1.6;//default_linear_ki * 16 / 10
      linear_pid1_.integ_limit_ = default_max_trans_vel_ * 0.6;
	  linear_pid1_.output_limit_ = default_max_trans_vel_ * 0.6;
	  max_trans_vel_ = default_max_trans_vel_ * 0.6;
	  angular_pid1_.kp_ = rot_kp_ + 0.2;
	  angular_pid1_.ki_ = 0.01;
	  std::cout<<"max v = "<<max_trans_vel_<<"k_i = "<< linear_pid1_.ki_ <<std::endl;
    }
    else if(level == 3)//average: 0.8, max: 1.1
    {
      linear_pid1_.kp_ = linear_kp_ + 0.1; 
      linear_pid1_.ki_ = linear_ki_ ;
      linear_pid1_.integ_limit_ = default_max_trans_vel_;
	  linear_pid1_.output_limit_ = default_max_trans_vel_;
	  angular_pid1_.kp_ = rot_kp_ + 0.2;
	  max_trans_vel_ = default_max_trans_vel_ + 0.1; 
	  std::cout<<"max v = "<<max_trans_vel_<<"k_i = "<< linear_pid1_.ki_ <<std::endl;
    }
    else
      ROS_ERROR("move base : setVelocity");
  	
    return;
}
void PurePersuitLocalPlanner::showPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
	int n = path.size();
	if(n<1)
		return;
	nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path.at(0).header.frame_id;
    gui_path.header.stamp = ros::Time(0);
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses.at(i) = path.at(i);
    }
	pub_path_.publish(gui_path);
	ros::spinOnce();
}
};