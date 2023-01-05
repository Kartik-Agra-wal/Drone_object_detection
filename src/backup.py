#!/usr/bin/env python

import numpy as np
import random
import math
import rospy
import roscpp
from   std_msgs.msg       import Float64
from   geometry_msgs.msg  import Twist
from   geometry_msgs.msg  import PoseStamped
from   drone_basic.msg  import quad_pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler


x_current = 0.0
y_current = 0.0
z_current = 0.0

quat_x  = 0.0
quat_y  = 0.0
quat_z  = 0.0
quat_w  = 0.0

roll  = 0.0
pitch = 0.0
yaw   = 0.0

PI = 3.14159265359

def pi2pi(angl):

    global PI
    
    while(abs(angl)-PI > 0.001):
        if(angl>PI):
            angl = angl-2*PI
        
        if (angl<-PI):
            angl = angl+2*PI

    return angl

def get_rotation(data1):
    global x_current, y_current, z_current, quat_x, quat_y, quat_z, quat_w, roll, pitch, yaw

    x_current  = data1.pose.position.x
    y_current  = data1.pose.position.y
    z_current  = data1.pose.position.z

    quat_x   = data1.pose.orientation.x 
    quat_y   = data1.pose.orientation.y
    quat_z   = data1.pose.orientation.z
    quat_w   = data1.pose.orientation.w

    orientation_list = [quat_x, quat_y, quat_z, quat_w]

    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

from sensor_msgs.msg import LaserScan



 

def laser_data_processing(data,temp):
    global region_left_min,region_mid_min,region_right_min
    regions={'left': [], 'mid':[],  'right':[]    }
    regions = { 'left' : data.ranges[0:360],'mid' : data.ranges[360:720], 'right' : data.ranges[720:1081] }
    try:
        region_left_min=min( min(regions['left']) , 6 )  
    except:
        region_left_min = 6
    try:
        region_mid_min=min( min(regions['left']) , 6 )  
    except:
        region_mid_min = 6
    try:
        region_right_min=min( min(regions['left']) , 6 )  
    except:
        region_right_min = 6
 

    # print(" // ",region_left_min , "// ",region_mid_min , " // ",region_right_min)


              
def main():

    global roll, pitch, yaw, x_current,y_current,z_current,PI

    waypoint_x = np.array([18.0,15, 0.0])*0.5
    waypoint_y = np.array([0.0 ,6, 0.0])*0.5
    
    goal_z = 1
    
    kp_z   = 0.2
    
    Kp_linear = 0.1
    Kd_linear = 0.05
    Ki_linear = 0.001   

    Kp_angular = 3
    Kd_angular = 0.1
    Ki_angular = 0.01

    flag = 0

    rospy.init_node('quad_waypoint2', anonymous=True)
    rate = rospy.Rate(10) # 5hz

    rospy.loginfo("This Node is working")

    sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, get_rotation)
    subscriber = rospy.Subscriber("/scan",LaserScan,laser_data_processing,10)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 15)
    pub_pose    = rospy.Publisher('/traj/pose', quad_pose, queue_size = 15)
    cmd =Twist()
    pose=quad_pose()

    while not rospy.is_shutdown():
        
        if(flag==0):
            rate.sleep()
            rate.sleep()
            for i in range(len(waypoint_x)):
                goal_x    = waypoint_x[i]
                goal_y    = waypoint_y[i]
                
                distanceC = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)))
                distanceP = distanceC
                
                angular_errorP = 0.0

                total_error         = 0.0
                total_angular_error = 0.0




                while abs(distanceC)>1:

                    if(region_left_min!=6 and region_mid_min!=6 and region_right_min!=6):
                        print("Case 111 : Take a U turn")
                        cmd.linear.x  = 0.0
                        cmd.angular.z = 0.8
                        pub_cmd_vel.publish(cmd)
                        continue
                    elif(region_left_min!=6 and region_mid_min==6 and region_right_min!=6):
                        print("Case 101 : Straight Movement")
                        cmd.linear.x  = 0.5
                        cmd.angular.z = 0.0
                        pub_cmd_vel.publish(cmd)
                        continue
                    elif(region_left_min==6 and region_mid_min!=6 and region_right_min!=6):
                        print("Case 011 : Take Left Turn")
                        cmd.linear.x  = 0.3
                        cmd.angular.z = 0.5
                        pub_cmd_vel.publish(cmd)
                        continue
                    elif(region_left_min!=6 and region_mid_min!=6 and region_right_min==6):
                        print("Case 110 : Take Right Turn")
                        cmd.linear.x  = 0.3
                        cmd.angular.z = -0.5
                        pub_cmd_vel.publish(cmd)
                        continue
                    elif(region_left_min==6 and region_mid_min==6 and region_right_min!=6):
                        print("Case 001 : Move Straight")
                        cmd.linear.x  = 0.5
                        cmd.angular.z = 0.0
                        pub_cmd_vel.publish(cmd)
                        continue
                    elif(region_left_min!=6 and region_mid_min==6 and region_right_min==6):
                        print("Case 100 : Move Straight")
                        cmd.linear.x  = 0.5
                        cmd.angular.z = 0.0
                        pub_cmd_vel.publish(cmd)
                        continue
                    # elif(region_left_min==6 and region_mid_min==6 and region_right_min==6):
                    #     print("Case 000 : Do not Move")
                    #     cmd.linear.x  = 0.0
                    #     cmd.angular.z = 0.0
                    #     pub_cmd_vel.publish(cmd)
                    
                    

                    distanceC     = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)))
                    linear_speed  = (distanceC * Kp_linear) + ((distanceC - distanceP)* Kd_linear) + (total_error*Ki_linear)
                    
                    error = goal_z - z_current


                    desired_angle_goal = math.atan2(goal_y-y_current, goal_x-x_current)
                    angular_errorC     = pi2pi(desired_angle_goal-yaw)
                    angular_speed      = (angular_errorC*Kp_angular) + ((angular_errorC-angular_errorP)*Kd_angular) + (total_angular_error*Ki_angular)

                    print('distance = {} angular error = {} '.format(distanceC, angular_errorC*180/PI))

                    cmd.linear.x  = linear_speed
                    cmd.angular.z = angular_speed
                    cmd.linear.z  = kp_z * error

                    distanceP      = distanceC
                    angular_errorP = angular_errorC

                    total_error         = total_error + distanceC
                    total_angular_error = total_angular_error + angular_errorC

                    pose.x = x_current
                    pose.y = y_current
                    pose.z = z_current

                    pose.roll  = roll
                    pose.pitch = pitch
                    pose.yaw   = yaw


                    pub_cmd_vel.publish(cmd)
                    pub_pose.publish(pose)
                    print('Waypoint = {}--[{},{}] '.format(i+1,goal_x,goal_y))
                    flag=1
                    rate.sleep()

            error = z_current-0.0
            print('Height error = {} '.format(error))
            while error>0.2:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                error         = z_current-0.0
                cmd.linear.z  = -kp_z * error

                pose.x = x_current
                pose.y = y_current
                pose.z = z_current

                pose.roll  = roll
                pose.pitch = pitch
                pose.yaw   = yaw

                pub_cmd_vel.publish(cmd)
                pub_pose.publish(pose)
                print('Height error = {} '.format(error))
                rate.sleep()
        else:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            cmd.linear.z  = 0.0
            pub_cmd_vel.publish(cmd)
            print('Quad Landed Sucessfully Covering all the Waypoints')
            rate.sleep()

    	
    rate.sleep()
    rospy.loginfo("Node is shutting down")

    # rospy.spin()

if __name__ == '__main__':
    main()