/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

    MoveBase::MoveBase(tf2_ros::Buffer& tf) :
        // 初始化 MoveBase 节点
        tf_(tf),
        as_(NULL),
        planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
        bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
        blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
        recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
        planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
        runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

        // move_base 的 action 服务器
        as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

        // 全局命名空间
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        // 恢复行为触发器
        recovery_trigger_ = PLANNING_R;     // enum move_base::RecoveryTrigger::PLANNING_R = 0

        // get some parameters that will be global to the move base node
        // 设置参数服务器 key-value 值，param(name,value,default)，没有则使用默认值
        std::string global_planner, local_planner;
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
        private_nh.param("planner_frequency", planner_frequency_, 0.0);
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

        // parameters of make_plan service
        // 路径规划服务所需的参数
        private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
        private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

        // set up plan triple buffer
        // 路径规划时用到的三个buffer
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();      // 规划计划
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();       // 最新计划
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();   // 控制计划

        // set up the planner's thread
        // 开启一个线程用来规划路径，说明 move_base 可以边运动边规划
        planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

        // for commanding the base
        // 申请用来控制的 cmd_vel 话题
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        // 申请用来标注当前目的地的话题
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

        // 创建了一个命名空间为 move_base 的句柄
        ros::NodeHandle action_nh("move_base");
        // 申请用来发布目的地的话题
        action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        // 申请用来发表恢复行为的话题
        recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

        // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
        // they won't get any useful information back about its status, but this is useful for tools
        // like nav_view and rviz
        // 主要是个rviz这些工具使用的部分，直接拿取并没有实际意义
        // 创建了一个命名空间为 move_base_simple 的句柄
        ros::NodeHandle simple_nh("move_base_simple");
        // 订阅 goal 话题信息
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

        // we'll assume the radius of the robot to be consistent with what's specified for the costmaps
        // 假设机器人半径和代价地图中设置的参数保持一致，指的是 costmap_common_params.yaml 这个文件中的设置
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

        // create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        // 创建代价地图规划器的封装类，并初始化
        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();
        // initialize the global planner
        // 初始化全局规划器
        try {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
            } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        // create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        // 创建代价地图规划器的封装类，并初始化
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->pause();
        // create a local planner
        // 初始化局部规划器
        try {
            tc_ = blp_loader_.createInstance(local_planner);
            ROS_INFO("Created local_planner %s", local_planner.c_str());
            tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
            } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }

        // Start actively updating costmaps based on sensor data
        // 开始根据传感器数据更新代价地图
        planner_costmap_ros_->start();      // 规划器
        controller_costmap_ros_->start();   // 控制器

        // advertise a service for getting a plan
        // 注册一个用于获得地图信息的服务
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

        // advertise a service for clearing the costmaps
        // 注册一个用于清空地图信息的服务
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

        // if we shutdown our costmaps when we're deactivated... we'll do that now
        // 如果手动终止了代价地图，那么下面的代码用来清理环境
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }

        // load any user specified recovery behaviors, and if that fails load the defaults
        // 加载任何用户指定的恢复行为，如果加载失败了则加载默认的恢复行为
        if(!loadRecoveryBehaviors(private_nh)){
            loadDefaultRecoveryBehaviors();
        }

        // initially, we'll need to make a plan
        // 在初始阶段，将状态机状态设置为规划 PLANNING 状态
        state_ = PLANNING;

        // we'll start executing recovery behaviors at the beginning of our list
        // 将恢复行为模式从0开始遍历，因为恢复行为可能有很多个，因此需要逐个遍历选用哪种恢复方式
        recovery_index_ = 0;

        // we're all set up now so we can start the action server
        // 到达这一步就可以开始了，启动action服务器
        as_->start();

        // 设置一个全局的动态参数服务器
        dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    // MoveBase 重置参数 callback 
    void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        // The first time we're called, we just want to make sure we have the original configuration
        // 第一次调用的时候只希望所有配置都是最初的设置
        if(!setup_)
        {
            last_config_ = config;
            default_config_ = config;
            setup_ = true;
            return;
        }

        // 如果需要恢复默认值则进行恢复
        if(config.restore_defaults) {
            config = default_config_;
            // if someone sets restore defaults on the parameter server, prevent looping
            // 如果有人在参数服务器上设置了默认配置，那么设置为false，防止死循环
            config.restore_defaults = false;
        }

        // 调整规划频率
        if(planner_frequency_ != config.planner_frequency)
        {
            planner_frequency_ = config.planner_frequency;
            p_freq_change_ = true;
        }

        // 调整控制频率
        if(controller_frequency_ != config.controller_frequency)
        {
            controller_frequency_ = config.controller_frequency;
            c_freq_change_ = true;
        }

        // 规划过程中的最大容忍时间
        planner_patience_ = config.planner_patience;
        // 在清理空间之前，控制器等待的最长时间
        controller_patience_ = config.controller_patience;
        // 重新规划路径的重复次数，一次规划可能会失败，因此路径规划操作可能会被执行很多次
        max_planning_retries_ = config.max_planning_retries;
        // 清空地图参数，清空多少m范围内的障碍物
        conservative_reset_dist_ = config.conservative_reset_dist;

        // 是否使用恢复机制
        recovery_behavior_enabled_ = config.recovery_behavior_enabled;
        // 是否启用转动机器人自身进行恢复的机制，必须建立在 recovery_behavior_enabled_ 可用的情况下
        clearing_rotation_allowed_ = config.clearing_rotation_allowed;
        // 在不活动的情况下是否终止 move_base 的代价地图
        shutdown_costmaps_ = config.shutdown_costmaps;

        // 震荡超时设置
        oscillation_timeout_ = config.oscillation_timeout;
        // 震荡距离
        oscillation_distance_ = config.oscillation_distance;

        // 如果当前的全局规划和最新的不符
        if(config.base_global_planner != last_config_.base_global_planner) {
            boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
            // initialize the global planner
            // 初始化全局规划器
            ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
            try {
                planner_ = bgp_loader_.createInstance(config.base_global_planner);

                // wait for the current planner to finish planning
                // 等待当前规划器完成规划
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

                // Clean up before initializing the new planner
                // 在初始化新的规划器之前清空先前规划好的路径
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

                lock.unlock();
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                        containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.base_global_planner = last_config_.base_global_planner;
            }
        }

        // 如果当前的局部规划和最新的不符
        if(config.base_local_planner != last_config_.base_local_planner){
            boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
            // create a local planner
            // 创建局部规划器
            try {
                tc_ = blp_loader_.createInstance(config.base_local_planner);
                // Clean up before initializing the new planner
                // 在初始化新的规划器之前清空先前规划好的路径
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                        containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
                tc_ = old_planner;
                config.base_local_planner = last_config_.base_local_planner;
            }
        }

        make_plan_clear_costmap_ = config.make_plan_clear_costmap;
        make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

        last_config_ = config;
    }

    // MoveBase 目的地 callback
    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
        ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;
        // 发布当前的action目的地
        action_goal_pub_.publish(action_goal);
    }

    // MoveBase 清空机器人周围障碍物
    void MoveBase::clearCostmapWindows(double size_x, double size_y){
        geometry_msgs::PoseStamped global_pose;

        // clear the planner's costmap
        // 清空规划器的代价地图
        getRobotPose(global_pose, planner_costmap_ros_);

        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

        // clear the controller's costmap
        // 清空控制器的代价地图
        getRobotPose(global_pose, controller_costmap_ros_);

        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
    }

    // MoveBase 清空代价地图服务器
    bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
        //clear the costmaps
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }

    // MoveBase 规划服务
    bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
        if(as_->isActive()){
            ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
            return false;
        }
        // make sure we have a costmap for our planner
        // 确保有一个代价地图可以被用来规划
        if(planner_costmap_ros_ == NULL){
            ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
            return false;
        }

        geometry_msgs::PoseStamped start;
        // if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
        // 如果用户没有指定初始姿态，那么用一个空的帧来识别机器人当前的姿态并使用
        if(req.start.header.frame_id.empty())
        {
            geometry_msgs::PoseStamped global_pose;
            if(!getRobotPose(global_pose, planner_costmap_ros_)){
                ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
                return false;
            }
            start = global_pose;
        }
        else
        {
            start = req.start;
        }

        if (make_plan_clear_costmap_) {
            // update the copy of the costmap the planner uses
            // 更新规划器所使用的代价地图一份拷贝
            clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
        }

        // first try to make a plan to the exact desired goal
        // 首先尝试去规划一个精准的目标
        std::vector<geometry_msgs::PoseStamped> global_plan;
        if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
            ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                req.goal.pose.position.x, req.goal.pose.position.y);

            // search outwards for a feasible goal within the specified tolerance
            // 生成式搜索可行的公差目的地
            geometry_msgs::PoseStamped p;
            p = req.goal;
            bool found_legal = false;
            float resolution = planner_costmap_ros_->getCostmap()->getResolution();
            float search_increment = resolution*3.0;
            // 
            if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
            // 逐步增大公差进行搜索
            for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
                for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
                    for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {
                        // don't search again inside the current outer layer
                        // 对已经搜索过的内层区域不再进行搜索
                        if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

                        // search to both sides of the desired goal
                        // 搜索目的地两侧
                        for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

                        // if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
                        // 
                        if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

                            for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                                if(planner_->makePlan(start, p, global_plan)){
                                    if(!global_plan.empty()){

                                        if (make_plan_add_unreachable_goal_) {
                                            //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                                            //(the reachable goal should have been added by the global planner)
                                            global_plan.push_back(req.goal);
                                        }

                                        found_legal = true;
                                        ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                        break;
                                    }
                                }
                                else{
                                ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                }
                            }
                        }
                    }
                }
            }
        }

        // copy the plan into a message to send out
        // 将规划好的信息打包起来并发布
        resp.plan.poses.resize(global_plan.size());
        for(unsigned int i = 0; i < global_plan.size(); ++i){
        resp.plan.poses[i] = global_plan[i];
        }

        return true;
    }

    // MoveBase 析构
    MoveBase::~MoveBase(){
        // 将恢复行为清空
        recovery_behaviors_.clear();

        delete dsrv_;

        if(as_ != NULL)
        delete as_;

        if(planner_costmap_ros_ != NULL)
        delete planner_costmap_ros_;

        if(controller_costmap_ros_ != NULL)
        delete controller_costmap_ros_;

        planner_thread_->interrupt();
        planner_thread_->join();

        delete planner_thread_;

        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;

        planner_.reset();
        tc_.reset();
    }

    // MoveBase 规划函数
    bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

        // make sure to set the plan to be empty initially
        // 在规划前清空之前的规划结果
        plan.clear();

        // since this gets called on handle activate
        // 确保规划函数在句柄被激活的时候调用
        if(planner_costmap_ros_ == NULL) {
            ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
            return false;
        }

        // get the starting pose of the robot
        // 获得机器人初始姿态
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)) {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
            return false;
        }

        const geometry_msgs::PoseStamped& start = global_pose;

        // if the planner fails or returns a zero length plan, planning failed
        // 如果规划失败则返回0长度的路径
        if(!planner_->makePlan(start, goal, plan) || plan.empty()){
        ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
        return false;
        }

        return true;
    }

    // MoveBase 发布停止控制
    void MoveBase::publishZeroVelocity(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }

    // MoveBase 检查四元数是否可用
    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
        // first we need to check if the quaternion has nan's or infs
        // 确保四元数可用
        if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        // next, we need to check if the length of the quaternion is close to zero
        // 检查四元数的长度是否为零
        if(tf_q.length2() < 1e-6){
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        // next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        // 归一化四元数，并检查其在竖直方向上是否正确变化
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if(fabs(dot - 1) > 1e-3){
            ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }

    // MoveBase 将目的地的坐标系转换成全局坐标系
    geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        geometry_msgs::PoseStamped goal_pose, global_pose;
        goal_pose = goal_pose_msg;

        // just get the latest available transform... for accuracy they should send goals in the frame of the planner
        // 只获取最新的可用坐标变化
        goal_pose.header.stamp = ros::Time();

        try{
            tf_.transform(goal_pose_msg, global_pose, global_frame);
        }
        catch(tf2::TransformException& ex){
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        return global_pose;
    }

    // MoveBase 唤醒规划器
    void MoveBase::wakePlanner(const ros::TimerEvent& event)
    {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }

    // MoveBase 用来规划的线程
    void MoveBase::planThread(){
        ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
        ros::NodeHandle n;
        ros::Timer timer;
        bool wait_for_wake = false;
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        while(n.ok()){
            // check if we should run the planner (the mutex is locked)
            // 如果需要进行规划
            while(wait_for_wake || !runPlanner_){
                //if we should not be running the planner then suspend this thread
                ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
                planner_cond_.wait(lock);
                wait_for_wake = false;
            }
            ros::Time start_time = ros::Time::now();

            // time to plan! get a copy of the goal and unlock the mutex
            // 进行规划，并放开锁
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

            // run planner
            planner_plan_->clear();
            bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

            if(gotPlan){
                ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
                // pointer swap the plans under mutex (the controller will pull from latest_plan_)
                std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

                lock.lock();
                planner_plan_ = latest_plan_;
                latest_plan_ = temp_plan;
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                new_global_plan_ = true;

                ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

                // make sure we only start the controller if we still haven't reached the goal
                // 确保在在启动控制器的时候机器人并没有到达目的地
                if(runPlanner_)
                state_ = CONTROLLING;
                if(planner_frequency_ <= 0)
                runPlanner_ = false;
                lock.unlock();
            }
            // if we didn't get a plan and we are in the planning state (the robot isn't moving)
            // 如果目前还没有得到规划结果，并且处于规划状态（机器人没有移动）
            else if(state_==PLANNING){
                ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
                ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

                // check if we've tried to make a plan for over our time limit or our maximum number of retries
                // issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
                // is negative (the default), it is just ignored and we have the same behavior as ever
                // 检查是否超过了最大规划时间和次数限制，如果超过了则忽略本次规划
                lock.lock();
                planning_retries_++;
                if(runPlanner_ &&
                (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
                    // we'll move into our obstacle clearing mode
                    // 进入清空障碍模式
                    state_ = CLEARING;
                    runPlanner_ = false;  // proper solution for issue #523
                    publishZeroVelocity();
                    recovery_trigger_ = PLANNING_R;
                }

                lock.unlock();
            }

            //take the mutex for the next iteration
            lock.lock();

            //setup sleep interface if needed
            if(planner_frequency_ > 0){
                ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
                if (sleep_time > ros::Duration(0.0)){
                wait_for_wake = true;
                timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
                }
            }
        }
    }

    // MoveBase 执行 callback 
    void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
    {
        // 如果四元数无效则直接返回
        if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
        }

        geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

        publishZeroVelocity();
        // we have a goal so start the planner
        // 当有一个目的地的时候开始规划
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        current_goal_pub_.publish(goal);

        ros::Rate r(controller_frequency_);
        if(shutdown_costmaps_){
        ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();
        }

        // we want to make sure that we reset the last time we had a valid plan and control
        // 确保我们重置了上次规划和控制的时间
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;

        ros::NodeHandle n;
        while(n.ok())
        {
            if(c_freq_change_)
            {
                ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
                r = ros::Rate(controller_frequency_);
                c_freq_change_ = false;
            }

            if(as_->isPreemptRequested()){
                if(as_->isNewGoalAvailable()){
                    // if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                    // 如果当前是激活状态并且有一个可行的目的地，那么接受这个目的地
                    move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

                    if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
                        return;
                    }

                    goal = goalToGlobalFrame(new_goal.target_pose);

                    // we'll make sure that we reset our state for the next execution cycle
                    // 确保为下一个执行循环重置了当前状态
                    recovery_index_ = 0;
                    state_ = PLANNING;

                    // we have a new goal so make sure the planner is awake
                    // 当有一个新的目的地时确保规划其是avtivate的
                    lock.lock();
                    planner_goal_ = goal;
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                    lock.unlock();

                    // publish the goal point to the visualizer
                    // 将目的地点发布在可视化工夹具上 rviz 
                    ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                    current_goal_pub_.publish(goal);

                    //make sure to reset our timeouts and counters
                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;
                }
                else {
                    // if we've been preempted explicitly we need to shut things down
                    // 如果明确当前抢占到资源，那么终止一切
                    resetState();

                    // notify the ActionServer that we've successfully preempted
                    // 通知 action 服务器我们已经成功抢占到资源
                    ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
                    as_->setPreempted();

                    //we'll actually return from execute after preempting
                    return;
                }
            }

            // we also want to check if we've changed global frames because we need to transform our goal pose
            if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
                goal = goalToGlobalFrame(goal);

                //we want to go back to the planning state for the next execution cycle
                recovery_index_ = 0;
                state_ = PLANNING;

                //we have a new goal so make sure the planner is awake
                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();

                //publish the goal point to the visualizer
                ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
                current_goal_pub_.publish(goal);

                //make sure to reset our timeouts and counters
                last_valid_control_ = ros::Time::now();
                last_valid_plan_ = ros::Time::now();
                last_oscillation_reset_ = ros::Time::now();
                planning_retries_ = 0;
            }

            // for timing that gives real time even in simulation
            ros::WallTime start = ros::WallTime::now();

            // the real work on pursuing a goal is done here
            bool done = executeCycle(goal);

            // if we're done, then we'll return from execute
            if(done)
                return;

            // check if execution of the goal has completed in some way

            ros::WallDuration t_diff = ros::WallTime::now() - start;
            ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

            r.sleep();
            //make sure to sleep for the remainder of our cycle time
            if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
        }

        // wake up the planner thread so that it can exit cleanly
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //if the node is killed then we'll abort and return
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
        return;
    }

    // MoveBase 计算距离函数
    double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
    {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }

    // MoveBase 执行循环
    bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal){
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        // we need to be able to publish velocity commands
        // 需要发布速度命令
        geometry_msgs::Twist cmd_vel;

        // update feedback to correspond to our curent position
        // 依据我们当前的位置来更新回调
        geometry_msgs::PoseStamped global_pose;
        getRobotPose(global_pose, planner_costmap_ros_);
        const geometry_msgs::PoseStamped& current_position = global_pose;

        // push the feedback out
        // 发布action的feedback
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_->publishFeedback(feedback);

        // check to see if we've moved far enough to reset our oscillation timeout
        // 检查，如果机器人已经开出了足够远说明没有产生抖动，重置抖动时间
        if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
            last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position;

            // if our last recovery was caused by oscillation, we want to reset the recovery index
            // 如果最新的恢复行为是因为抖动引起的，那么需要选用新的恢复行为
            if(recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        // check that the observation buffers for the costmap are current, we don't want to drive blind
        // 检查代价地图的缓冲区是否会是最新的，因为不能盲开
        if(!controller_costmap_ros_->isCurrent()){
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }

        // if we have a new plan then grab it and give it to the controller
        // 如果有一个新的规划，那么把这个发给控制器
        if(new_global_plan_){
            // make sure to set the new plan flag to false
            // 确保最新的规划flag是fakse
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

            // do a pointer swap under mutex
            std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            controller_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base","pointers swapped!");

            if(!tc_->setPlan(*controller_plan_)){
                //ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                //disable the planner thread
                lock.lock();
                runPlanner_ = false;
                lock.unlock();

                as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
                return true;
            }

            //make sure to reset recovery_index_ since we were able to find a valid plan
            if(recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }

        // the move_base state machine, handles the control logic for navigation
        // move_base 的状态机
        switch(state_){
            // if we are in a planning state, then we'll attempt to make a plan
            // 如果当前处于规划状态，那么尝试进行一次规划
            case PLANNING:
                {
                    boost::recursive_mutex::scoped_lock lock(planner_mutex_);
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                }
                ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
                break;

            // if we're controlling, we'll attempt to find valid velocity commands
            // 如果当前是控制状态，那么尝试找到一个可行的速度控制命名
            case CONTROLLING:
                ROS_DEBUG_NAMED("move_base","In controlling state.");

                // check to see if we've reached our goal
                // 检查是否已经到达目的地
                if(tc_->isGoalReached()){
                    ROS_DEBUG_NAMED("move_base","Goal reached!");
                    resetState();

                    //disable the planner thread
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                    return true;
                }

                // check for an oscillation condition
                // 检查是否出现震荡的情况
                if(oscillation_timeout_ > 0.0 &&
                    last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
                {
                    publishZeroVelocity();
                    state_ = CLEARING;
                    recovery_trigger_ = OSCILLATION_R;
                }

                {
                    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

                    if(tc_->computeVelocityCommands(cmd_vel)){
                    ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                    last_valid_control_ = ros::Time::now();
                    // make sure that we send the velocity command to the base
                    // 确保将速度控制命令发送到机器人底盘
                    vel_pub_.publish(cmd_vel);
                    if(recovery_trigger_ == CONTROLLING_R)
                        recovery_index_ = 0;
                    }
                    else {
                        ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                        ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                        // check if we've tried to find a valid control for longer than our time limit
                        // 检查当前控制命令是否超过了时间限制
                        if(ros::Time::now() > attempt_end){
                            // we'll move into our obstacle clearing mode
                            // 进入清空障碍物的状态
                            publishZeroVelocity();
                            state_ = CLEARING;
                            recovery_trigger_ = CONTROLLING_R;
                        }
                        else{
                            // otherwise, if we can't find a valid control, we'll go back to planning
                            // 否则，如果找不到一个有效的控制，那么重新回到规划模式
                            last_valid_plan_ = ros::Time::now();
                            planning_retries_ = 0;
                            state_ = PLANNING;
                            publishZeroVelocity();

                            // enable the planner thread in case it isn't running on a clock
                            // 如果没有在运行，那么让规划器线程运行
                            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                            runPlanner_ = true;
                            planner_cond_.notify_one();
                            lock.unlock();
                        }
                    }
                }

                break;

            // we'll try to clear out space with any user-provided recovery behaviors
            // CLEARING 模式，尝试使用任何用户提供的恢复行为来清空周围空间
            case CLEARING:
                ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
                // we'll invoke whatever recovery behavior we're currently on if they're enabled
                // 如果进入了当前恢复行为，那么直接调用
                if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
                    ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

                    move_base_msgs::RecoveryStatus msg;
                    msg.pose_stamped = current_position;
                    msg.current_recovery_number = recovery_index_;
                    msg.total_number_of_recoveries = recovery_behaviors_.size();
                    msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

                    recovery_status_pub_.publish(msg);

                    recovery_behaviors_[recovery_index_]->runBehavior();

                    //we at least want to give the robot some time to stop oscillating after executing the behavior
                    last_oscillation_reset_ = ros::Time::now();

                    //we'll check if the recovery behavior actually worked
                    ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
                    last_valid_plan_ = ros::Time::now();
                    planning_retries_ = 0;
                    state_ = PLANNING;

                    //update the index of the next recovery behavior that we'll try
                    recovery_index_++;
                }
                else{
                    ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
                    //disable the planner thread
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

                    if(recovery_trigger_ == CONTROLLING_R){
                        ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
                    }
                    else if(recovery_trigger_ == PLANNING_R){
                        ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
                    }
                    else if(recovery_trigger_ == OSCILLATION_R){
                        ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
                    }
                    resetState();
                    return true;
                }
                break;

            default:
                ROS_ERROR("This case should never be reached, something is wrong, aborting");
                resetState();
                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();
                as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
                return true;
        }

        //we aren't done yet
        return false;
    }

    // MoveBase 加载恢复行为函数
    bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
        XmlRpc::XmlRpcValue behavior_list;
        if(node.getParam("recovery_behaviors", behavior_list)){
            if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
                for(int i = 0; i < behavior_list.size(); ++i){
                    if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                        if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
                            // check for recovery behaviors with the same name
                            // 检查恢复行为是否存在重名
                            for(int j = i + 1; j < behavior_list.size(); j++){
                                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                                    if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                                        std::string name_i = behavior_list[i]["name"];
                                        std::string name_j = behavior_list[j]["name"];
                                        if(name_i == name_j){
                                        ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                            name_i.c_str());
                                        return false;
                                        }
                                    }
                                }
                            }
                        }
                        else{
                            ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                            return false;
                        }
                    }
                    else{
                        ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                            behavior_list[i].getType());
                        return false;
                    }
                }

                // if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
                // 如果到达这一步说明所有恢复行为形式符合规范的，对这些行为进行创建
                for(int i = 0; i < behavior_list.size(); ++i){
                    try{
                        // check if a non fully qualified name has potentially been passed in
                        // 检查是否传入了非完全限定名
                        if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
                            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                            for(unsigned int i = 0; i < classes.size(); ++i){
                                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                                //if we've found a match... we'll get the fully qualified name and break out of the loop
                                ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                    std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                behavior_list[i]["type"] = classes[i];
                                break;
                                }
                            }
                        }

                        boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                        // shouldn't be possible, but it won't hurt to check
                        if(behavior.get() == NULL){
                            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                            return false;
                        }

                        // initialize the recovery behavior with its name
                        // 根据其名字初始化恢复行为
                        behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
                        recovery_behavior_names_.push_back(behavior_list[i]["name"]);
                        recovery_behaviors_.push_back(behavior);
                    }
                    catch(pluginlib::PluginlibException& ex){
                        ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                        return false;
                    }
                }
            }
            else{
                ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                    behavior_list.getType());
                return false;
            }
        }
        else{
        // if no recovery_behaviors are specified, we'll just load the defaults
        // 如果用户没有指定恢复行为，那么使用默认的
        return false;
        }

        // if we've made it here... we've constructed a recovery behavior list successfully
        // 如果到了此处，活命已经成功建立了恢复行为函数的列表
        return true;
    }

    // we'll load our default recovery behaviors here
    // MoveBase 加载默认的恢复行为
    void MoveBase::loadDefaultRecoveryBehaviors(){
        recovery_behaviors_.clear();
        try{
            // we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
            // 需要根据传入的参数进行设置，以保证向下兼容
            ros::NodeHandle n("~");
            n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
            n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

            // first, we'll load a recovery behavior to clear the costmap
            // 首先加载用来清空代价地图的恢复行为
            boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("conservative_reset");
            recovery_behaviors_.push_back(cons_clear);

            // next, we'll load a recovery behavior to rotate in place
            // 然后加载用于原地旋转的恢复行为
            boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
            if(clearing_rotation_allowed_){
                rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
                recovery_behavior_names_.push_back("rotate_recovery");
                recovery_behaviors_.push_back(rotate);
            }

            // next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            // 然后加载用于更加激进的恢复行为函数
            boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("aggressive_reset");
            recovery_behaviors_.push_back(ags_clear);

            // we'll rotate in-place one more time
            // 原地再旋转一次
            if(clearing_rotation_allowed_){
                recovery_behaviors_.push_back(rotate);
                recovery_behavior_names_.push_back("rotate_recovery");
            }
        }
        catch(pluginlib::PluginlibException& ex){
            ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }

    // MoveBase 重置状态
    void MoveBase::resetState(){
        // Disable the planner thread
        // 停止规划线程
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // Reset statemachine
        // 重置状态机
        state_ = PLANNING;
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocity();

        // if we shutdown our costmaps when we're deactivated... we'll do that now
        // 关闭代价地图
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }

    // MoveBase 获取机器人姿态
    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time(); // latest available
        ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

        // get robot pose on the given costmap frame
        // 在已知的代价地图中获得机器人姿态
        try
        {
            tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException& ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        // 检查全局姿态时间戳是否在坐标变化的可容忍范围内
        if (!global_pose.header.stamp.isZero() &&
            current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
        {
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                                "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                                current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }

        return true;
    }
};
