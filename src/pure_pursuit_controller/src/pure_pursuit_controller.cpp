#include <pure_pursuit_controller/pure_pursuit_controller.h>

#include <cmath>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cassert>


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

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

geometry_msgs::PoseStamped PurePursuitController::getCurrentPose()
const
{
    geometry_msgs::PoseStamped robot_origin_pose, robot_world_pose;

    robot_origin_pose.header.frame_id = pose_frame_id_;
    robot_origin_pose.pose.orientation.w = 1;

    //ROS_INFO(" Value : %f, %f, %f, %f ", robot_origin_pose.pose.orientation.x, robot_origin_pose.pose.orientation.y, robot_origin_pose.pose.orientation.z, robot_origin_pose.pose.orientation.w);
    assert(!(robot_origin_pose.pose.orientation.x == 0 && robot_origin_pose.pose.orientation.y == 0 && robot_origin_pose.pose.orientation.z == 0 && robot_origin_pose.pose.orientation.w == 0));
    transformToWorld(robot_origin_pose, robot_world_pose);

    //ROS_INFO("Robot in world coordinates %f, %f, %f ", robot_world_pose.pose.position.x , robot_world_pose.pose.position.y, tf::getYaw(robot_world_pose.pose.orientation));
    return robot_world_pose;
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
}

void PurePursuitController::spin()
{
    ros::spin();
}

void PurePursuitController::timerCallback(const ros::TimerEvent& event)
{
    if (status_ != PurePursuitController::STATUS::EXECUTING) {
        return;
    }

    //ROS_INFO("In timerCallback");
    geometry_msgs::Twist cmd_vel;

    if (getCommandVelocity(cmd_vel)) {
        ROS_INFO("Sending %f , %f \n", cmd_vel.linear.x, cmd_vel.angular.z);
        cmd_vel_pub_.publish(cmd_vel);

        const size_t numPoints = 20;

        double look_ahead_th = getLookAheadThreshold();
        visualization_msgs::Marker cmd_traj;

        cmd_traj.header.frame_id = pose_frame_id_;
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
            pose.position.x = cmd_vel.linear.x * std::cos(
                                  pose.orientation.z) * dt;
            pose.position.y = cmd_vel.linear.x * std::sin(
                                  pose.orientation.z) * dt;

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


bool PurePursuitController::getCommandVelocity(geometry_msgs::Twist &twist)
{
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;

    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    /*
        for(int k=0;k<cur_ref_path_.poses.size();k++)
        {
          visualization_msgs::Marker marker;
          marker.header.frame_id = ref_path_frame_id_;
          marker.header.stamp = ros::Time();
          marker.ns = "my_namespace";
          marker.id = 0;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = cur_ref_path_.poses[k].pose.position.x;
          marker.pose.position.y = cur_ref_path_.poses[k].pose.position.y;
          marker.pose.position.z = cur_ref_path_.poses[k].pose.position.z;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = 0.1;
          marker.scale.y = 0.1;
          marker.scale.z = 0.1;
          marker.color.a = 1.0; // Don't forget to set the alpha!
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker_array.markers.push_back(marker);
        }
        goal_point_pub_.publish( marker_array );
        */

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
            //if (1) {
            //  pose = cur_ref_path_.poses[next_waypoint_];
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
            if (std::abs(std::sin(look_ahead_angle)) >= epsilon_) {
                double radius = 0.5 * (look_ahead_dist / std::sin(look_ahead_angle));

                double linear_vel = velocity_;
                if (std::abs(radius) >= epsilon_)
                    angular_vel = linear_vel / radius;

                twist.linear.x = linear_vel;
                twist.angular.z = angular_vel;
                //ROS_INFO(" Advancing");
                return true;
            }
        }
    }
    //ROS_INFO(" Not advancing");
    return false;
}

double PurePursuitController::getLookAheadDistance(const geometry_msgs::PoseStamped &target_pose) const
{

    geometry_msgs::PoseStamped robot_world_pose = getCurrentPose();



    tf::Vector3 v1(target_pose.pose.position.x,
                   target_pose.pose.position.y,
                   target_pose.pose.position.z);
    tf::Vector3 v2(robot_world_pose.pose.position.x,
                   robot_world_pose.pose.position.y,
                   robot_world_pose.pose.position.z);
    //ROS_INFO("robot_world_pose %s   target_pose %s ",robot_world_pose.header.frame_id.c_str(),target_pose.header.frame_id.c_str());
    assert(strcmp(robot_world_pose.header.frame_id.c_str(), target_pose.header.frame_id.c_str()) == 0);
    return tf::tfDistance(v1, v2);
}

double PurePursuitController::getLookAheadAngle(const geometry_msgs::PoseStamped &target_pose) const
{
    geometry_msgs::PoseStamped robot_heading_vector, transformed_target_pose;
    robot_heading_vector.header.frame_id = pose_frame_id_;
    robot_heading_vector.pose.position.x = 1;
    robot_heading_vector.pose.position.y = 0;
    robot_heading_vector.pose.position.z = 0;
    robot_heading_vector.pose.orientation.x = 0;
    robot_heading_vector.pose.orientation.y = 0;
    robot_heading_vector.pose.orientation.z = 0;
    robot_heading_vector.pose.orientation.w = 1;

    try {
        tf_listenner_.transformPose(pose_frame_id_, target_pose, transformed_target_pose);
    } catch (tf::TransformException &exception) {
        ROS_ERROR_STREAM("PurePursuitController::getLookAheadDistance: " << exception.what());
        return -1.0;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = pose_frame_id_;
    marker.header.stamp = ros::Time();
    marker.ns = "heading_vector_maker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    marker.points.resize(2);
    geometry_msgs::Point point1, point2;
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;

    point2.x = robot_heading_vector.pose.position.x;
    point2.y = robot_heading_vector.pose.position.y;
    point2.z = robot_heading_vector.pose.position.z;

    marker.points[0] = (point1);
    marker.points[1] = (point2);
    heading_vecotr_pub_.publish(marker);

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = pose_frame_id_;
    marker2.header.stamp = ros::Time();
    marker2.ns = "look_ahead_marker";
    marker2.id = 0;
    marker2.type = visualization_msgs::Marker::ARROW;
    marker2.action = visualization_msgs::Marker::ADD;

    marker2.pose.position.x = 0;
    marker2.pose.position.y = 0;
    marker2.pose.position.z = 0;
    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;

    marker2.scale.x = 0.1;
    marker2.scale.y = 0.1;
    marker2.scale.z = 0.1;
    marker2.color.a = 1.0; // Don't forget to set the alpha!
    marker2.color.r = 1.0;
    marker2.color.g = 0.0;
    marker2.color.b = 0.0;

    marker2.points.resize(2);
    point1.x = 0;
    point1.y = 0;
    point1.z = 0;

    point2.x = transformed_target_pose.pose.position.x;
    point2.y = transformed_target_pose.pose.position.y;
    point2.z = transformed_target_pose.pose.position.z;

    marker2.points[0] = (point1);
    marker2.points[1] = (point2);
    look_ahead_pub_.publish(marker2);



    tf::Vector3 v1(robot_heading_vector.pose.position.x,
                   robot_heading_vector.pose.position.y,
                   robot_heading_vector.pose.position.z);
    tf::Vector3 v2(transformed_target_pose.pose.position.x,
                   transformed_target_pose.pose.position.y,
                   transformed_target_pose.pose.position.z);
    //ROS_INFO("robot_heading_vector %s   transformed_target_pose %s ",robot_heading_vector.header.frame_id.c_str(),transformed_target_pose.header.frame_id.c_str());
    assert(strcmp(robot_heading_vector.header.frame_id.c_str(), transformed_target_pose.header.frame_id.c_str()) == 0);

    double angle = std::atan2(v2.y(), v2.x()) - std::atan2(v1.y(), v1.x());
    ROS_INFO("Angle is %f ", angle);
    return angle;//tf::tfAngle(v1, v2);
}

double PurePursuitController::getLookAheadThreshold() const
{
    //ROS_INFO("Computing LookAheadThreshold ratio %f , vel %f ", lookahead_ratio_,cur_vel_.linear.x);
    return lookahead_ratio_ * cur_vel_.linear.x + 0.5;
}
/*
  double PurePursuitController::getArcDistance(const
      geometry_msgs::PoseStamped& pose) const {
    double look_ahead_dist = getLookAheadDistance(pose);
    double look_ahead_angle = getLookAheadAngle(pose);

    if (std::abs(std::sin(look_ahead_angle)) >= epsilon_)
    {
      ROS_INFO("Computing getArcDistance is %f ", look_ahead_dist/sin(look_ahead_angle)*look_ahead_angle);
      return look_ahead_dist/sin(look_ahead_angle)*look_ahead_angle;
    }
    else
    {
      ROS_INFO("Computing getArcDistance is %f ", look_ahead_dist);
      return look_ahead_dist;
    }
  }
*/
int PurePursuitController::getNextWayPoint(int wayPoint) const
{

    if (!cur_ref_path_.poses.empty()) {
        if (next_waypoint_ >= 0) {
            geometry_msgs::PoseStamped origin = getCurrentPose();
            tf::Vector3 v_1(origin.pose.position.x,
                            origin.pose.position.y,
                            origin.pose.position.z);
            double look_ahead_th = getLookAheadThreshold();
            ROS_INFO("LookAheadThreshold is %f ", look_ahead_th);
            for (int i = next_waypoint_; i < cur_ref_path_.poses.size();
                    ++i) {
                tf::Vector3 v_2(cur_ref_path_.poses[i].pose.position.x,
                                cur_ref_path_.poses[i].pose.position.y,
                                cur_ref_path_.poses[i].pose.position.z);

                if (tf::tfDistance(v_1, v_2) > look_ahead_th) {
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
/*
int PurePursuitController::getClosestWayPoint() const {
  if (!cur_ref_path_.poses.empty()) {
    int closestWaypoint = -1;
    double minDistance = -1.0;

    for (int i = 0; i < cur_ref_path_.poses.size(); ++i) {
      double distance = getArcDistance(cur_ref_path_.poses[i]);

      if ((minDistance < 0.0) || (distance < minDistance)) {
        closestWaypoint = i;
        minDistance = distance;
      }
    }
    ROS_INFO("Closest waypoint id is %d", closestWaypoint);
    return closestWaypoint;
  }
  ROS_INFO("Closest waypoint id is %d", -1);
  return -1;
}
*/

bool PurePursuitController::getInterpolatedPose(int wayPoint, geometry_msgs::PoseStamped &interpolated_pose) const
{
    //ROS_INFO("In interpolated_pose: Got frame id %s  \n", interpolated_pose.header.frame_id.c_str());
    if (!cur_ref_path_.poses.empty()) {
        if (wayPoint > 0) {
            ROS_INFO("Waypoint > 0");
            double l_t = getLookAheadThreshold();
            double p_t = getLookAheadDistance(
                             cur_ref_path_.poses[next_waypoint_ - 1]);
            //       ROS_INFO(" LookAheadDistance is %f. LookAheadThreshold is %f ", p_t, l_t);
            if (p_t < l_t) {
                ROS_INFO("Waypoint > 0");
                geometry_msgs::PoseStamped p_0 = getCurrentPose();
                geometry_msgs::PoseStamped p_1 =
                    cur_ref_path_.poses[wayPoint - 1];
                geometry_msgs::PoseStamped p_2 =
                    cur_ref_path_.poses[wayPoint];

                tf::Vector3 v_1(p_2.pose.position.x - p_0.pose.position.x,
                                p_2.pose.position.y - p_0.pose.position.y,
                                p_2.pose.position.z - p_0.pose.position.z);
                tf::Vector3 v_2(p_1.pose.position.x - p_0.pose.position.x,
                                p_1.pose.position.y - p_0.pose.position.y,
                                p_1.pose.position.z - p_0.pose.position.z);
                tf::Vector3 v_0(p_2.pose.position.x - p_1.pose.position.x,
                                p_2.pose.position.y - p_1.pose.position.y,
                                p_2.pose.position.z - p_1.pose.position.z);

                double l_0 = v_0.length();
                double l_1 = v_1.length();
                double l_2 = v_2.length();

                v_0.normalize();
                v_2.normalize();

                double alpha_1 = M_PI - tf::tfAngle(v_0, v_2);
                double beta_2 = asin(l_2 * sin(alpha_1) / l_t);
                double beta_0 = M_PI - alpha_1 - beta_2;
                double l_s = l_2 * sin(beta_0) / sin(beta_2);
                tf::Vector3 p_s(p_1.pose.position.x + v_0[0]*l_s,
                                p_1.pose.position.y + v_0[1]*l_s,
                                p_1.pose.position.z + v_0[2]*l_s);

                interpolated_pose.pose.position.x = p_s[0];
                interpolated_pose.pose.position.y = p_s[1];
                interpolated_pose.pose.position.z = p_s[2];

                interpolated_pose.pose.orientation.x = 0;
                interpolated_pose.pose.orientation.y = 0;
                interpolated_pose.pose.orientation.z = 0;
                interpolated_pose.pose.orientation.w = 1;

                interpolated_pose.header.frame_id = ref_path_frame_id_;
                //interpolated_pose = cur_ref_path_.poses[wayPoint-1];
                //          ROS_INFO("Interpolated pose is valid");
                return true;
            }
        }

        interpolated_pose = cur_ref_path_.poses[wayPoint];
        interpolated_pose.header.frame_id = ref_path_frame_id_;
        //ROS_INFO("\nEnd of interpolated_pose: Got frame id %s  \n", interpolated_pose.header.frame_id.c_str());
        return true;
    }
    //ROS_INFO("Interpolated pose is not valid");
    return false;
}

void PurePursuitController::transformToWorld(const geometry_msgs::PoseStamped &input_pose, geometry_msgs::PoseStamped &output_pose) const
{
    //ROS_INFO("TXFORM POSE FROM %s to %s", input_pose.header.frame_id.c_str(), ref_path_frame_id_.c_str());
    assert(!(input_pose.pose.orientation.x == 0 && input_pose.pose.orientation.y == 0 && input_pose.pose.orientation.z == 0 && input_pose.pose.orientation.w == 0));
    try {
        tf_listenner_.transformPose(ref_path_frame_id_,
                                  input_pose, output_pose);
    } catch (tf::TransformException &exception) {
        ROS_ERROR_STREAM("PurePursuitController::transformToWorld: " <<
                         exception.what());
    }
    //ROS_INFO("OUTPUT POSE IS %s", output_pose.header.frame_id.c_str());

}

void PurePursuitController::getParameters()
{
    nh_.param<int> ("ros/queue_depth", queue_size_, 100);
    nh_.param<std::string> ("ros/path_topic_name", path_topic_name_, "/reference_path");
    nh_.param<std::string> ("ros/odometry_topic_name", odom_topic_name_, "/odometry/filtered");
    nh_.param<std::string> ("ros/cmd_vel__topic_name", cmd_vel_topic_, "/huskyvelocity__controller/cmd_vel");
    nh_.param<std::string> ("ros/cmd_trajectory_topic_name", cmd_traj_topic_, "/local_planner_solution_trajectory");
    nh_.param<std::string> ("ros/pose_frame_id", pose_frame_id_, "base_link");

    nh_.param<double> ("controller/frequency", frequency_, 20.0);
    nh_.param<int> ("controller/initial_waypoint", initial_waypoint_, -1);
    nh_.param<double> ("controller/velocity", velocity_, 0.2);
    nh_.param<double> ("controller/look_ahead_ratio", lookahead_ratio_, 1.0);
    nh_.param<double> ("controller/epsilon", epsilon_, 1e-6);
}

