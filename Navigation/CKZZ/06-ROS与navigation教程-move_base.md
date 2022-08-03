## ROS与navigation教程-move_base

原文链接 [ROS与navigation教程-move_base](https://www.ncnynl.com/archives/201708/1898.html)

* move_base 为 actionlib 提供了实现途径，将尝试控制机器人从一个地点运动到另外一个地点。
* move_base 节点将全局路径规划和局部路径规划链接到一起，以完成全局导航任务。
* move_base 支持符合 nav_core 指定的 nav_core::BaseGlobalPlanner 规划器插件。
* move_base 维护了两个代价地图，一个用于全局路径规划，一个用于局部路径规划。

![ROS move_base](../images/nav-move-base-01.png)

-----

在进行下一步之前需要了解一些常用 ROS 黑话：

| sentence | meaning | 
|----|----|
|max_trans_vel|最大平移速度|
|max_rot_vel|最大旋转速度|
|planner_frequency|全局规划操作的执行频率，如果设置为0.0,则全局规划器仅在接收到新的目标点或者局部规划器报告路径堵塞时才会重新执行规划操作。|
|controller_frequency|底盘控制移动话题cmd_vel发送命令的频率|
|planner_patience|路径规划最大容忍时间|
|controller_patience|在执行clean之前，控制器等到的最长时间|
|max_planning_retries|重新规划的次数，一次规划可能会失败，因此规划可能被执行多次|
|conservative_reset_dist|清空地图参数，清空多少m范围内的障碍物|
|recovery_behavior_enabled|是否启动恢复机制|
|clearing_rotation_allowed|是否启用转动机器人自身进行恢复的机制，必须建立在 recovery_behavior_enabled_ 可用的情况下|

-----

### move_base 节点运行流程

![move_base node working flow](../images/nav-move-base-02.png)

上图中的运行流程如下所示：

1. 导航时如果内阻塞了 stuck，那么以保守的方式进行恢复 Conservative Reset；
2. 如果保守方式恢复失败则以忽视障碍物并旋转的方式恢复 Clearing Rotation；
3. 如果旋转方式恢复失败则以激进的方式恢复 Aggressive Reset；
4. 如果激进方式恢复失败则再忽视障碍物并旋转的方式恢复 Clearing Rotation；
5. 如果此时还是失败则直接终止 Aborted；

实际上机器人是按照下面的步骤运行的：
1. move_base 节点让机器人能够到达允许的误差范围内的目的地（如偏差0.5m）。
2. 在没有障碍物的局部地图情况下，move_base 的运行结果要么接近了目的地，要么远离。
3. 当机器人认为自己卡住的时候，move_base 提供了一些列恢复行为来帮助机器人重新规划，这些恢复行为可以是默认的也可以是自定义符合 MoveBase 规范的插件。

其中默认的恢复行为主要执行了以下步骤：
* 将局部代价地图中的障碍物全部清除，然后重新使用激光雷达进行扫描构建新的局部代价地图。通常情况下这就可以继续运行了，因为里程计偏移可能让机器人误以为进入了障碍物区域。
* 如果清理后仍然在障碍物区域，那么会将激光雷达能够扫描到的区域障碍物全部清除，并就地旋转来新建这一片区域。
* 然后再旋转一次构建地图来检查自身状态。



这个图实际上就是 navigation 部分的 **状态机**，源码可以在这个包中的 [resources/move_base.cpp](./../resources/move_base.cpp)获得，也可以查看 github 链接诶 [navigation/move_base/src/move_base.cpp](https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/src/move_base.cpp)。

move_base 类的详细文档 [move_base::MoveBase Class Reference](http://docs.ros.org/en/diamondback/api/move_base/html/classmove__base_1_1MoveBase.html#_details)

这个文件的总代码量为1200行，算是非常短的文件，推荐逐行仔细阅读，在这个demo的resource文件下存放了个人阅读笔记。

以下是状态机的详细内容，状态机是move_base中非常重要的内容，务必弄清楚其运行步骤：
```cpp
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
```

