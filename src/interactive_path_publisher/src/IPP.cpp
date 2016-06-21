#include <interactive_path_publisher/IPP.h>

IPP::IPP ( ros::NodeHandle nh )
{
  nh_ = nh;
  pointSub_ = nh.subscribe ( "clicked_point",10,&IPP::onReceivePoint,this );
  pathPub_ = nh.advertise<nav_msgs::Path> ( "ref_path",10 );

  tolerance_ = 0.3;
  reset();
}

void IPP::onReceivePoint ( const geometry_msgs::PointStamped::ConstPtr& msg )
{
  ROS_INFO ( "Got point" );
  if ( status_ == IPP::STATUS::READY )
    {
      Eigen::Vector2d point ( msg->point.x,msg->point.y );

      bool add = false;
      if ( !points_.empty() )
        {
          Eigen::Vector2d diff = point - points_.back();
          if ( diff.norm() > tolerance_ )
            {
              add = true;
            }
        }
      else
        {
          add = true;
        }

      //only add the point if it is not too close. this is a hack to overcome lack of UI support in rviz
      if ( add == true )
        {
          points_.push_back ( point );
        }
      else
        {
          status_ = IPP::STATUS::DONE;
        }
    }

}

IPP::STATUS IPP::getStatus() const
{
  return status_;
}
void IPP::reset()
{
  points_.clear();
  status_ = IPP::STATUS::READY;
}


void IPP::publishPath()
{
  nav_msgs::Path path;
  std::string frameId = "odom";
  path.header.frame_id = frameId.c_str();
  for ( int i = 0 ; i < points_.size() ; i++ )
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;
      pose.header.frame_id = frameId.c_str();
      pose.pose.position.x = points_[i][0];
      pose.pose.position.y = points_[i][1];
      path.poses.push_back ( pose );
    }
  pathPub_.publish ( path );
  ROS_INFO ( "IPP : Published path" );
}

