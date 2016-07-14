#include <pure_pursuit_controller/pure_pursuit_controller.h>


PurePursuitController::PurePursuitController(const
        ros::NodeHandle &nh)
{
    nh_ = nh;
    getParameters();

    path_sub_ = nh_.subscribe(path_topic_name_, queue_size_, &PurePursuitController::pathCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_name_, queue_size_, &PurePursuitController::odometryCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist> ( cmd_vel_topic_, queue_size_);
    cmd_traj_pub_ = nh_.advertise<visualization_msgs::Marker> (cmd_traj_topic_, queue_size_);
    goal_point_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("goal_point", queue_size_);
    heading_vecotr_pub_ = nh_.advertise<visualization_msgs::Marker> ("heading_vector", queue_size_);
    look_ahead_pub_ = nh_.advertise<visualization_msgs::Marker> ("look_ahead", queue_size_);

    timer_ = nh.createTimer(ros::Duration(1.0 / frequency_), &PurePursuitController::timerCallback, this);

    status_ = PurePursuitController::STATUS::INITIALIZED;
    reset();
}

PurePursuitController::~PurePursuitController()
{

}


void PurePursuitController::pathCallback(const nav_msgs::Path &msg)
{
    if (status_ == PurePursuitController::STATUS::EXECUTING) {
        reset(); // reset first and then transfer control to block below
    }

    if (status_ == PurePursuitController::STATUS::READY) {
        cur_ref_path_ = msg;
        ref_path_frame_id_ = msg.header.frame_id;
        next_waypoint_ = -1;

        status_ = PurePursuitController::STATUS::EXECUTING;

        //ROS_INFO("The given reference path frame id is %s ", ref_path_frame_id_.c_str());
    }

}

void PurePursuitController::odometryCallback(const nav_msgs::Odometry& msg)
{
    cur_vel_ = msg.twist.twist;
    // cur_odom_ = msg;
}

void PurePursuitController::timerCallback(const ros::TimerEvent& event)
{
    if (status_ != PurePursuitController::STATUS::EXECUTING) {
        return;
    }

    //ROS_INFO("In timerCallback");
    geometry_msgs::Twist cmd_vel;

    if (getNextCmdVel(cmd_vel)) {
        ROS_INFO("Sending %f , %f \n", cmd_vel.linear.x, cmd_vel.angular.z);
        cmd_vel_pub_.publish(cmd_vel);

        const size_t numPoints = 20;

        double look_ahead_th = getLookAheadThreshold();
        visualization_msgs::Marker cmd_traj;

        cmd_traj.header.frame_id = body_frame_id_;
        cmd_traj.header.stamp = ros::Time::now();
        cmd_traj.ns = "solution_trajectory";
        cmd_traj.type = 4;
        cmd_traj.action = 0;
        cmd_traj.scale.x = 0.12;
        cmd_traj.color.r = 0.0;
        cmd_traj.color.g = 0.0;
        cmd_traj.color.b = 1.0;
        cmd_traj.color.a = 1.0;
        cmd_traj.lifetime = ros::Duration(0);
        cmd_traj.frame_locked = true;
        cmd_traj.pose = geometry_msgs::Pose();
        cmd_traj.points.resize(numPoints);

        for (int i = 0; i < numPoints; ++i) {
            geometry_msgs::Pose pose;
            double dt = look_ahead_th * (double) i / (double) numPoints;

            pose.orientation.z = cmd_vel.angular.x * dt;
            pose.position.x = cmd_vel.linear.x * std::cos(pose.orientation.z) * dt;
            pose.position.y = cmd_vel.linear.x * std::sin(pose.orientation.z) * dt;

            cmd_traj.points[i] = pose.position;
        }


        cmd_traj_pub_.publish(cmd_traj);
    }
}

void PurePursuitController::reset()
{
    cur_ref_path_.poses.clear();
    next_waypoint_ = -1;
    status_ = PurePursuitController::STATUS::READY;
}


bool PurePursuitController::getNextCmdVel(geometry_msgs::Twist &twist)
{
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;

    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    ROS_INFO(" Current waypoint number %d ", next_waypoint_);
    next_waypoint_ = getNextWayPoint(next_waypoint_);

    ROS_INFO(" Next waypoint number %d ", next_waypoint_);

    if (next_waypoint_ == cur_ref_path_.poses.size() - 1) {
        ROS_INFO("Final point %d ", next_waypoint_);
        status_ = PurePursuitController::STATUS::DONE;
        reset();
        return true;
    }

    if (next_waypoint_ >= 0) {
        geometry_msgs::PoseStamped pose;
        ROS_INFO(" Waypoint number %d ", next_waypoint_);

        if (getInterpolatedPose(next_waypoint_, pose)) {

            visualization_msgs::MarkerArray marker_array;

            visualization_msgs::Marker marker;
            marker.header.frame_id = ref_path_frame_id_;
            marker.header.stamp = ros::Time();
            marker.ns = "goal_point_maker";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = pose.pose.position.x;
            marker.pose.position.y = pose.pose.position.y;
            marker.pose.position.z = pose.pose.position.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
            goal_point_pub_.publish(marker_array);

            double look_ahead_dist = getLookAheadDistance(pose);
            double look_ahead_angle = getLookAheadAngle(pose);

            double angular_vel = 0.0;
            double linear_vel = target_velocity_;

            if (std::abs(std::sin(look_ahead_angle)) >= epsilon_) {
                double radius = 0.5 * (look_ahead_dist / std::sin(look_ahead_angle));


                if (std::abs(radius) >= epsilon_)
                    // this is omega = V_set * curvature
                    angular_vel = linear_vel / radius;

                twist.linear.x = linear_vel;
                twist.angular.z = angular_vel;
                //ROS_INFO(" Advancing");
                return true;
            } else {
                // look ahead angle is too small, so just moving forward
                twist.linear.x = linear_vel;
                twist.angular.z = 0;
            }
        }
    }
    //ROS_INFO(" Not advancing");
    return false;
}

double PurePursuitController::getLookAheadDistance(const geometry_msgs::PoseStamped &target_pose) const
{

    tf::Vector3 v1(target_pose.pose.position.x,
                   target_pose.pose.position.y,
                   target_pose.pose.position.z);

    //ROS_INFO("robot_world_pose %s   target_pose %s ",robot_world_pose.header.frame_id.c_str(),target_pose.header.frame_id.c_str());
    assert(strcmp(robot_world_pose.header.frame_id.c_str(), target_pose.header.frame_id.c_str()) == 0);

    return tf::tfDistance(v1, cur_pose_.getRotation());
}

double PurePursuitController::getLookAheadAngle(const geometry_msgs::PoseStamped &target_pose) const
{
    tf::Quaternion target_angle_qt;
    quaternionMsgToTF(target_pose.orientation, target_angle_qt);
    // angleShortestPath();
    double angle = tf::angle(cur_pose_.getRotation(), target_angle_qt).getYaw();
    return angle;
}

double PurePursuitController::getLookAheadThreshold() const
{
    //ROS_INFO("Computing LookAheadThreshold ratio %f , vel %f ", lookahead_ratio_,cur_vel_.linear.x);
    return lookahead_ratio_ * cur_vel_.linear.x + 0.5;
}

int PurePursuitController::getNextWayPoint(int wayPoint) const
{

    if (!cur_ref_path_.poses.empty()) {
        if (next_waypoint_ >= 0) {
            double look_ahead_th = getLookAheadThreshold();
            ROS_INFO("LookAheadThreshold is %f ", look_ahead_th);
            for (int i = next_waypoint_; i < cur_ref_path_.poses.size(); ++i) {
                // construct a tf vector with the i th point
                tf::Vector3 v_2(cur_ref_path_.poses[i].pose.position.x,
                                cur_ref_path_.poses[i].pose.position.y,
                                cur_ref_path_.poses[i].pose.position.z);
                // check if the distance is higher than the lookahead threshold
                if (tf::tfDistance(cur_pose_.getOrigin(), v_2) > look_ahead_th) {
                    ROS_INFO("Next waypoint id is %d ", i);
                    return i;
                }
            }
            ROS_INFO("Next waypoint id is %d ", next_waypoint_);
            return next_waypoint_;
        } else {
            ROS_INFO("Next waypoint id is %d (since given waypoint id was < 0)", 0);
            return 0;
        }
    }
    ROS_INFO("Next waypoint id is %d (since we are done)", -1);
    return -1;
}


/**
 * @brief      Thread to get the robot pose from TF.
 */
void PurePursuitController::getRobotPoseTFthread()
{
    ros::NodeHandle n;
    ros::Rate r(control_frequency_);
    ROS_INFO_ONCE("Setup TF thread, reading tf pose");
    while (n.ok()) {
        std::string error;
        tf::StampedTransform transform;
        try
        {
            ros::Time now = ros::Time::now();
            tf_.waitForTransform(robot_namespace_ + odom_frame_id_,
                                 robot_namespace_ + body_frame_id_,
                                 ros::Time(),
                                 ros::Duration(1.0));
            tf_.lookupTransform(robot_namespace_ + odom_frame_id_,
                                robot_namespace_ + body_frame_id_,
                                ros::Time(),
                                transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("tf failure: %s", ex.what());
            continue;
        }
        cur_pose_ = transfrom;
        // // get position
        // cur_pose_.position.x = transform.getOrigin().getX();
        // cur_pose_.position.y = transform.getOrigin().getY();
        // cur_pose_.position.z = transform.getOrigin().getZ();
        // cur_pose_.orientation = transfrom.getOrientation();
        // // get angle
        // tf::Matrix3x3 m(transform.getRotation());
        // m.getRPY(theta_x_, theta_y_, theta_z_);
        r.sleep();

    }
}


void PurePursuitController::getParameters()
{
    nh_.param<int>         ("ros/queue_depth",         queue_size_,      100);
    nh_.param<std::string> ("ros/body_frame_id",       body_frame_id_,   "base_link");
    nh_.param<std::string> ("ros/odom_frame_id",       odom_frame_id_,   "base_link");
    nh_.param<std::string> ("ros/path_topic_name",     path_topic_name_, "/reference_path");
    nh_.param<std::string> ("ros/odom_topic_name",     odom_topic_name_, "/odometry/filtered");
    nh_.param<std::string> ("ros/cmd_traj_topic_name", cmd_traj_topic_,  "/local_planner_solution_trajectory");
    nh_.param<std::string> ("ros/cmd_vel_topic_name",  cmd_vel_topic_,   "/huskyvelocity__controller/cmd_vel");

    nh_.param<double> ("controller/frequency",        frequency_,         20.0);
    nh_.param<int>    ("controller/initial_waypoint", initial_waypoint_,  -1);
    nh_.param<double> ("controller/velocity",         target_velocity_,          0.2);
    nh_.param<double> ("controller/look_ahead_ratio", lookahead_ratio_,   1.0);
    nh_.param<double> ("controller/epsilon",          epsilon_,           1e-6);
}

