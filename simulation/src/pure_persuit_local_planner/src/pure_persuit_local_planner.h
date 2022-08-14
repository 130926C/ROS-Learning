#ifndef pure_persuit_local_planner_H
#define pure_persuit_local_planner_H
#include <nav_core/base_local_planner.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Int32.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <boost/thread/mutex.hpp>
#include <std_msgs/UInt16.h>
#include <ros/ros.h>
#include <costmap_2d/footprint.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <Pid.h>
#include <chrono>
#include <thread>//std
namespace pure_persuit_local_planner 
{
	class PurePersuitLocalPlanner: public nav_core::BaseLocalPlanner
	{
		public:
			PurePersuitLocalPlanner();
			virtual ~PurePersuitLocalPlanner();
			bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
			void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
			bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
			void setVelocity(int level);
		protected:
			bool isTrajectoryFeasible(	base_local_planner::CostmapModel* costmap_model,
										const std::vector<geometry_msgs::PoseStamped>& path,
										int start_id,
										int look_ahead_idx, 
										int &obstacle_index);
			bool isGoalReached();
			bool getTransformed2TargetPosition(	const geometry_msgs::PoseStamped& pose,
												const std::string& frame_id,
												double& x, double& y, double& theta); 
			bool checkGoalArrived(const std::vector<geometry_msgs::PoseStamped>& current_plan, double scale = 1.0);
			bool checkGoalArrived(const geometry_msgs::PoseStamped& final_goal, double scale);
		    int findStartPoseID(const std::vector<geometry_msgs::PoseStamped>& plan, int start_id = 0);
		    bool findLookforwardPoseIDs(int& start_id, 
		    							int& lookforward_id,
										const std::vector<geometry_msgs::PoseStamped>& plan);
		    bool transformPose2TargetFrame(const geometry_msgs::PoseStamped& pose,
											geometry_msgs::PoseStamped& result,
											const std::string& target_frame);
		    bool computeVelUsingPID(geometry_msgs::Twist& cmd,
									 const geometry_msgs::PoseStamped& temp_goal,
									 geometry_msgs::PoseStamped& temp_goal_pre,
									 double max_v,double max_w);
		    bool transformGlobalPlan( const tf::TransformListener* tf,
		      						  const std::vector<geometry_msgs::PoseStamped>& global_plan,
		      						  const std::string& global_frame,
		      						  std::vector<geometry_msgs::PoseStamped>& result_plan);
			// bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
			bool checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::PoseStamped& goal);
			void resetCarrot(geometry_msgs::PoseStamped& pre_temp_goal,
							const std::vector<geometry_msgs::PoseStamped>& current_plan);
			bool updateCarrot(	int& id,
								geometry_msgs::PoseStamped& temp_goal,
								geometry_msgs::PoseStamped& temp_goal_pre,
								const std::vector<geometry_msgs::PoseStamped>& current_plan);
			void odomCB(const nav_msgs::Odometry::ConstPtr& ptr);
			void laneEdgeCB(const std_msgs::Int32::ConstPtr& ptr);
			bool EnsureCurrentPlan(geometry_msgs::PoseStamped& goal, geometry_msgs::PoseStamped& start);
			void resetAllParam();
			bool slowStop(geometry_msgs::Twist& cmd_vel);
			void showPath(const std::vector<geometry_msgs::PoseStamped>& path);
			bool transform_pose(std::string parent_frame_id, 
                 				std::string child_frame_id, 
                 				const geometry_msgs::Pose& input_pose, 
                 				geometry_msgs::Pose& output_pose);
			std::vector<geometry_msgs::PoseStamped> current_plan_copy_;
			ros::Publisher lethal_points_pub_;
			ros::Publisher footprint_pub_;
			ros::Publisher carrot_pub_, start_pub_;
			ros::Publisher pub_path_;
			tf::TransformListener* tf_listener_;
			tf2_ros::Buffer* tf_;
			double p_precision_, o_precision_;
			double linear_kp_, linear_ki_, linear_kd_, rot_kp_, rot_ki_, rot_kd_;
			double default_max_trans_vel_, max_trans_vel_, max_rot_vel_;
			double lookingforward_dist_ ;
			double start_distance_err_;
			int goal_reached_, final_goal_reached_;
			int plan_idx_, start_idx_;
			geometry_msgs::PoseStamped robot_pose_;
			typedef boost::recursive_mutex mutex_t;
			bool slow_down_speed;
		private:
			boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
			const double robot_inscribed_radius_ = 0.1; //!< The radius of the inscribed circle of the robot (collision possible)
			mutex_t access_;
			mutex_t access_v_;
			mutex_t access_level_;
			mutex_t access_edge_;
			costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
			costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
			std::vector<geometry_msgs::PoseStamped> orig_global_plan_;
		    Pid linear_pid1_;
			Pid angular_pid1_;
			bool reset_carrot_;
			double current_robot_v_, current_robot_w_;
			ros::Subscriber odom_sub_;
			ros::Subscriber lane_edge_sub_;
			int edge_state_;
			bool started_;
			bool can_go_ ;
			int unvalid_counter_;
			bool use_map_3d_;
	};
}	//	namespace pure_persuit_local_planner
PLUGINLIB_EXPORT_CLASS(pure_persuit_local_planner::PurePersuitLocalPlanner, nav_core::BaseLocalPlanner)
#endif