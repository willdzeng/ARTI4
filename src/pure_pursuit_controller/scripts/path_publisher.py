#!/usr/bin/env python

# Reference paper : A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment
import yaml
import rospy
from math import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf



waypoint = [];

def init_waypoints(filename):
    global waypoint
    with open(filename, 'r') as ymlfile:
        cfg = yaml.load(ymlfile)
    waypoint = cfg["states"]
    rospy.loginfo("Loaded waypoints from file")

def main():
    global waypoint

    rospy.init_node('reference_path_node', anonymous=True)
    rospy.loginfo("Started reference path publisher")
    

    pub = rospy.Publisher('reference_path', Path, queue_size=10)

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        init_waypoints("/home/rpradeep/Documents/catkin_ws/src/my_scripts/trajgen/waypoints.yml")
        pth = Path()
        pth.header.frame_id = "odom"
        rospy.loginfo("Got %d waypoints."%(len(waypoint[0])))
        for k in range(0,len(waypoint[0])):
            x = waypoint[0][k];
            y = waypoint[1][k];
            ang = waypoint[2][k];

            ps = PoseStamped();
            ps.pose.position.x = x;
            ps.pose.position.y = y;
            ps.pose.position.z = 0;

            qz = tf.transformations.quaternion_about_axis(ang, (0,0,1));
            ps.pose.orientation.x = qz[0];
            ps.pose.orientation.y = qz[1];
            ps.pose.orientation.z = qz[2];
            ps.pose.orientation.w = qz[3];

            ps.header.frame_id = "odom" # Important
            print("\n X : %f , Y : %f , A : %f \n"%(x,y,ang))
            print("\n x : %f , y : %f , z : %f , w : %f\n"%(qz[0],qz[1],qz[2],qz[3]))

            pth.poses.append(ps);
        pub.publish(pth)
        rate.sleep()

    rospy.loginfo("About to exit waypoint scheduler")

    rate.sleep()

if __name__ == '__main__':
    try:
        
        main()
    except rospy.ROSInterruptException:
        pass
