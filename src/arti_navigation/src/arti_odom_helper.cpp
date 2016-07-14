#include <arti_navigation/odometry_helper_ros.h>

namespace arti_navigation {

ArtiOdomHelper::ArtiOdomHelper(std::string odom_topic) {
    setOdomTopic( odom_topic );
}

ArtiOdomHelper::ArtiOdomHelper(std::string base_link_name, std::string odom_link_name) {
    // setOdomTopic( odom_topic );
    setLinkNames(base_link_name, odom_link_name);
}

void ArtiOdomHelper::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("odom received!");

    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    base_odom_.child_frame_id = msg->child_frame_id;
//  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
//      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}

//copy over the odometry information
void ArtiOdomHelper::getOdom(nav_msgs::Odometry& base_odom) {
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom = base_odom_;
}


void ArtiOdomHelper::getRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;
    {
        boost::mutex::scoped_lock lock(odom_mutex_);
        global_vel.linear.x = base_odom_.twist.twist.linear.x;
        global_vel.linear.y = base_odom_.twist.twist.linear.y;
        global_vel.angular.z = base_odom_.twist.twist.angular.z;

        robot_vel.frame_id_ = base_odom_.child_frame_id;
    }
    robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
    robot_vel.stamp_ = ros::Time();
}

void ArtiOdomHelper::setOdomTopic(std::string odom_topic)
{
    if ( odom_topic != odom_topic_ )
    {
        odom_topic_ = odom_topic;

        if ( odom_topic_ != "" )
        {
            ros::NodeHandle gn;
            odom_sub_ = gn.subscribe<nav_msgs::Odometry>( odom_topic_, 1, boost::bind( &ArtiOdomHelper::odomCallback, this, _1 ));
        }
        else
        {
            odom_sub_.shutdown();
        }
    }
}

void ArtiOdomHelper::unsubscribeTopic() {
    odom_sub_.shutdown();
}

void ArtiOdomHelper::unsubscribeTF(){

}

void ArtiOdomHelper::subscribeTF(){

}

void ArtiOdomHelper::getRobotPoseTFthread()
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

} // end of namespace

