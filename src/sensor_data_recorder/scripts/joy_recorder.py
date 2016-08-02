#!/usr/bin/env python
from time import sleep
import os
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from husky_py_api.msg import DifferentialCmd
from time import sleep
import roslaunch
import pygame
import os



class AutoRecorder:
    user_name = "umdugv"
    use_diff_cmd = True
    diff_cmd_scale = 100
    vel_step = 0.1
    diff_step = 10
    def __init__(self):
        rospy.init_node('auto_recorder')
        self.cmd_pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=1)
        self.diff_pub = rospy.Publisher('/husky/cmd_diff', DifferentialCmd, queue_size=1)
        node = roslaunch.core.Node('sensor_data_recorder', 'sensor_data_recorder_node')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        pygame.init()
        r = rospy.Rate(10)
        # Joystick connection
        try:
            print "\nConnecting to joystick..."
            if pygame.joystick.get_count() < 1:
                print "No joystick found, quitting"
                sys.exit(1)
            else:
                real_joy = None
                joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
                print "Press <start> button to enable joy"
                while not rospy.is_shutdown():
                    pygame.init()
                    for joy in joysticks:
                        joy.init()
                        for event in pygame.event.get():
                            if event.type == pygame.QUIT: break
                        if joy.get_button(3):
                            real_joy = joy
                            break
                    if real_joy is not None:
                        joy = real_joy
                        print "Joystick found"
                        break
                    r.sleep()
        except:
            print "Can't find joy"
            return

        count = 0
        vx = 0
        wz = 0
        left_cmd = 0
        right_cmd = 0
        recording = False
        process = None
        print "Type your password to enable sound:"
        os.system("sudo modprobe pcspkr")
        print "Joy Stick Recorder is running"
        while not rospy.is_shutdown():
            # get button even first
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    break
            # if L2 pressed pass the joy command
            if joy.get_button(8):
                if self.use_diff_cmd:
                    left_tmp = -joy.get_axis(1) * self.diff_cmd_scale
                    right_tmp = -joy.get_axis(3) * self.diff_cmd_scale
                    cmd_diff = self.initDiffCmd(left_tmp,right_tmp)
                    self.diff_pub.publish(cmd_diff)
                else:
                    vx_tmp = -joy.get_axis(1)
                    wz_tmp = -joy.get_axis(0)
                    twist = self.initTwist(vx_tmp,wz_tmp)
                    self.cmd_pub.publish(twist)
                r.sleep()
                continue
            # if R2 pressed start recording
            if joy.get_button(9) and not recording:
                recording = True
                print "Start a new recording"
                count += 1
                if self.use_diff_cmd:
                    file_name = "/home/%s/%d_%0.3f_%0.3f.csv"%(self.user_name,count,left_cmd,right_cmd)
                else:
                    file_name = "/home/%s/%d_%0.3f_%0.3f.csv"%(self.user_name,count,vx,wz)
                rospy.set_param('/sensor_data_recorder/output_file',file_name)
                rospy.set_param('/sensor_data_recorder/odom_topic','/husky/odom')
                process = launch.launch(node)
                os.system("beep -f 600 -l 1000")
                sleep(0.5)
                # r.sleep()
                continue
            # if R2 keep pressed publish the system command
            if joy.get_button(9) and recording:
                if self.use_diff_cmd:
                    cmd_diff = self.initDiffCmd(left_cmd,right_cmd)
                    self.diff_pub.publish(cmd_diff)
                else:
                    twist = self.initTwist(vx,wz)
                    self.cmd_pub.publish(twist)
                r.sleep()
                continue
            # if R1 pressed stop the recording
            if joy.get_button(11):
                recording = False
                if process is not None:
                    process.stop()
                os.system("beep -f 500 -l 1000")
                sleep(0.5)
                continue
            # if "up" pressed increase left command by 10or vx by 0.1
            if joy.get_button(4):
                if self.use_diff_cmd:
                    left_cmd += self.diff_step
                    if left_cmd > 100:
                        left_cmd = 100
                    print "Left command increases to %f"%left_cmd
                else:
                    vx += self.vel_step
                    print "Lieanr Velocity increases to %f"%vx
                os.system("beep -f 400 -l 200")
                sleep(0.2)
                continue

            # if "down" pressed decrease left command by 10 or vx by 0.1
            if joy.get_button(6):
                if self.use_diff_cmd:
                    left_cmd -= self.diff_step
                    if left_cmd < -100:
                        left_cmd = -100
                    print "Left command decease to %f"%left_cmd
                else:
                    vx -= self.vel_step
                    print "Lieanr Velocity decease to %f"%vx
                os.system("beep -f 300 -l 200")
                sleep(0.2)
                continue

            # if "right" pressed increase right command by 10 or wz by 0.1
            if joy.get_button(5):
                if self.use_diff_cmd:
                    right_cmd += self.diff_step
                    if right_cmd > 100:
                        right_cmd = 100
                    print "Right command increases to %f"%right_cmd
                else:
                    wz += self.vel_step
                    print "Angular Velocity inrcrease to %f"%wz
                os.system("beep -f 400 -l 200")
                sleep(0.2)
                continue
                
            # if "left" pressend decrese right command by 10 or wz by 0.1
            if joy.get_button(7):
                if self.use_diff_cmd:
                    right_cmd -= self.diff_step
                    if right_cmd < -100:
                        right_cmd = -100
                    print "Right command decease to %f"%right_cmd
                else:
                    wz -= self.vel_step
                    print "Angular Velocity decease to %f"%wz
                os.system("beep -f 300 -l 200")
                sleep(0.2)
                continue

            if joy.get_button(3):
                if self.use_diff_cmd:
                    left_cmd = -left_cmd
                    right_cmd = -right_cmd
                    print "Velocit fliped to %f %f"%(left_cmd,right_cmd)
                else:
                    vx = -vx
                    wz = -wz
                    print "Velocit fliped to %f %f"%(vx,wz)
                os.system("beep -f 700 -l 300")
                sleep(0.2)
                continue

            if joy.get_button(0):
                if self.use_diff_cmd:
                    left_cmd = 0
                    right_cmd = 0
                else:
                    vx = 0
                    wz = 0
                print "Reset Velocit to 0"
                os.system("beep -f 350 -l 200")
                sleep(0.2)
                continue

            if self.use_diff_cmd:
                diffcmd = self.initDiffCmd(0,0)
                self.diff_pub.publish(diffcmd)
            else:
                twist = self.initTwist(0,0)
                self.cmd_pub.publish(twist)
            r.sleep()

        print "\ndata generation succeed exit clean"
        if process is not None:
            process.stop()


    def initTwist(self,vx,wz):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = wz
        return twist

    def initDiffCmd(self,left,right):
        diffcmd = DifferentialCmd()
        diffcmd.left_input = left
        diffcmd.right_input = right
        diffcmd.header.stamp = rospy.Time.now()
        return diffcmd


# END OF CLASS AUTO RECORDER
    
def main():
    AutoRecorder()


if __name__ == '__main__':
    main()


