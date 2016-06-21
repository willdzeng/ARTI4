#include <interactive_path_publisher/IPP.h>

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "IPPNode" );

  ros::NodeHandle n;

  IPP ipp ( n );

  ros::Rate loopRate ( 10 );

  while ( ros::ok() )
    {
      if ( ipp.getStatus() == IPP::STATUS::DONE )
        {
          ipp.publishPath();
          ipp.reset();
        }

      ros::spinOnce();
      loopRate.sleep();
    }

  return 0;
}
