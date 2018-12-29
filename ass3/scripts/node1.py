#!/usr/bin/env python
import rospy
import tf
import math
import random
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import sys
from geometry_msgs.msg import Twist, Pose ,Quaternion
from sensor_msgs.msg import LaserScan ,Image, CameraInfo
from nav_msgs.msg import Odometry
from image_geometry import PinholeCameraModel


PI = 3.1415926535897

print_msg = """Enter task number:
1 - Move forward
2 - Turn around
3 - Distance to object with color X
4 - Find object with color X"""



def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def turn_helper(alpha,clockwise):
    turn_publisher = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
    turn_msg = Twist()

    #turning speed
    turn_msg.linear.x = 0.0
    turn_msg.linear.y = 0.0
    turn_msg.linear.z = 0.0
    turn_msg.angular.x = 0.0
    turn_msg.angular.y = 0.0
    if clockwise:
        turn_msg.angular.z = -1.0
    else:
        turn_msg.angular.z = 1.0
    current_angle = 0.0
    relative_angle = (float(alpha))*2*PI/360

    while not rospy.is_shutdown():
        
        t0 = rospy.Time.now().to_sec()
        #start turnning untill the robot gets to the angle we want
        while((current_angle <= relative_angle) and (not rospy.is_shutdown())):
            turn_publisher.publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = 1.0*(t1-t0)
        #stoping the robot from turning
        turn_msg.angular.z = 0.0
        turn_publisher.publish(turn_msg)
        rospy.sleep(0.5)
        return
    
    

def forward():
    
    #checking obstacles between +15 and -15 degrees
    LIDAR_ERR = 0.05
    msg = rospy.wait_for_message("scan", LaserScan)
    scan_filter = []
    for i in range(360):
        if i <= 15 or i > 335:
            if msg.ranges[i] >= LIDAR_ERR:
		scan_filter.append(msg.ranges[i])
    #print(scan_filter)
    if min(scan_filter) > 0.6:
   	# Starts a new node
    	velocity_publisher = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
   	vel_msg = Twist()

   	#Since we are moving just in x-axis
    	vel_msg.linear.x = 0.1
    	vel_msg.linear.y = 0.0
   	vel_msg.linear.z = 0.0
    	vel_msg.angular.x = 0.0
    	vel_msg.angular.y = 0.0
   	vel_msg.angular.z = 0.0


   	while not rospy.is_shutdown():

       		#Setting the current time for distance calculus
       		t0 = rospy.Time.now().to_sec()
        	current_distance = 0

 	        #Loop to move the turtle in an specified distance
  	        while(current_distance < 0.40 and (not rospy.is_shutdown())):
           		#Publish the velocity
           		velocity_publisher.publish(vel_msg)
            		#Takes actual time to velocity calculus
           		t1=rospy.Time.now().to_sec()
           		#Calculates distancePoseStamped
            		current_distance= 0.1*(t1-t0)
       		#After the loop, stops the robot by braking slowly
                vel_msg.linear.x = 0.09
        	
        	velocity_publisher.publish
        	rospy.sleep(0.2)
                vel_msg.linear.x = 0.08
        	
        	velocity_publisher.publish(vel_msg)
                rospy.sleep(0.2)
                vel_msg.linear.x = 0.07
        	
        	velocity_publisher.publish(vel_msg)
                rospy.sleep(0.2)
                vel_msg.linear.x = 0.06
 
         	velocity_publisher.publish(vel_msg)
                rospy.sleep(0.2)
                vel_msg.linear.x = 0.05
                
        	velocity_publisher.publish(vel_msg)
        	rospy.sleep(0.2)
                vel_msg.linear.x = 0.04
                
        	velocity_publisher.publish(vel_msg)
        	rospy.sleep(0.2)
                vel_msg.linear.x = 0.03
                
        	velocity_publisher.publish(vel_msg)
        	rospy.sleep(0.2)
                vel_msg.linear.x = 0.02
                
        	velocity_publisher.publish(vel_msg)
        	rospy.sleep(0.2)
                vel_msg.linear.x = 0.01
                
        	velocity_publisher.publish(vel_msg)
        	rospy.sleep(0.2)
                vel_msg.linear.x = 0.0
                
        	velocity_publisher.publish(vel_msg)
        	rospy.sleep(0.5)


        	vel_msg.linear.x = 0.0
        	
        	velocity_publisher.publish(vel_msg)
		rospy.sleep(0.5)
		#return 1 when there is no obstacle and forward succeeded
		return 1
    
    print("There is an obstacle in the way!")
    rospy.sleep(0.5)
    #returns 0 if there is an obstacle
    return 0

def turn():
    print "Please enter an angle to turn"
    alpha = raw_input()
    turn_helper(alpha,True)
    return
    


def distance_helper(color):
    #getting the image from the camera and getting the color
    image_msg = rospy.wait_for_message("usb_cam/image_raw", Image)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    gaus_blur = cv2.GaussianBlur(cv_image,(3,3),0)
    cv2.imshow('image',cv_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    boundaries= {'blue': ([170,140,90],[255,170,120]),
                  'red': ([0,30,120],[50,100,255]),
                  'yellow': ([5,170,170],[80,254,254]),
                  'green' : ([75,130,90],[110,255,110]),
                  'purple': ([90,30,90],[150,90,170])}
    #    boundaries= {'blue': ([80,0,0],[255,60,60]),
    #              'red': ([0,0,80],[30,30,255]),
    #              'yellow': ([5,170,170],[80,254,254]),
     #             'green' : ([0,80,0],[80,255,60]),
    #              'purple': ([90,30,90],[150,90,170])}
    lower = np.array(boundaries[color][0], dtype = "uint8")
    upper = np.array(boundaries[color][1], dtype = "uint8")    
    mask = cv2.inRange(gaus_blur, lower, upper)

    kernelOpen = np.ones((5,5))
    kernelClose = np.ones((20,20))
    maskOpen = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernelOpen)
    final_mask = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
    cv2.imshow('finalmask',final_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    _,contours,h = cv2.findContours(final_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if contours == []:
        rospy.sleep(0.5)
        #return none if the color is not in the frame of the camera
        print "None"
        return None
    cnt = contours[0]
    M = cv2.moments(cnt)
    #calculating the distance and degree in 3D
    print " M['m00'] is %s" %M['m00']
    center_x = int(M['m10']/M['m00'])
    #print "center_x is %s" %center_x
    center_y = 240#int(M['m01']/M['m00'])
    
    center_of_image = (320,240)
    camInfo = rospy.wait_for_message("usb_cam/camera_info", CameraInfo)
    camera= PinholeCameraModel()
    #print " cam info is "
    #print camInfo
    camera.fromCameraInfo(camInfo)
    #print "before"
    #print "center is %s %s" %(center_x,center_y)
    ray_pixel = camera.projectPixelTo3dRay((center_x,center_y))
    #print "after"
    ray_center = camera.projectPixelTo3dRay(center_of_image)
    
    angle = math.acos(dotproduct(ray_pixel, ray_center) / (length(ray_pixel) * length(ray_center)))
    #print("angle is %s " %angle)
    degree = int(math.degrees(angle))

    
    if center_x>= center_of_image[0]:
        degree = 360-degree

    LIDAR_ERR = 0.05
    laser_msg = rospy.wait_for_message("scan", LaserScan)
    scan_filter = []
    #print "the degree is "
    #print (degree)
    print "the distance is "
    if(degree!= 360):
        print (laser_msg.ranges[degree])
        rospy.sleep(0.5)
        return (degree,laser_msg.ranges[degree])
    else:
       print (laser_msg.ranges[0])
       rospy.sleep(0.5)
       return (degree,laser_msg.ranges[0])
    

def distance():
    print "Please enter a color"
    color = raw_input()
    distance_helper(color)



def find_object_helper(color):
    #search in a loop -  in every 90 degrees, and if not found go forward 50 cm and do it again
    # if gets to an obstacle aboid it from the right by turning 90 degree and moving forward
    found = False
    distance_to_color = None
    noObstacle = True
    while not found:
        for i in range(4):
            turn_helper(90,True)
            distance_to_color = distance_helper(color)
            if distance_to_color is not None:
                found = True
                break
        
        if not found:
            noObstacle =  forward()
            while not noObstacle:
                turn_helper(90,True)
                noObstacle =  forward()
                if(noObstacle):
                    find_object_helper(color)
                    return
    
    if distance_to_color[0] == 0:
        turn_helper(distance_to_color[0],True)
    else:
        if 360-distance_to_color[0]>180:
            turn_helper(distance_to_color[0],False)
        else:
            turn_helper(360-distance_to_color[0],True)
    while distance_to_color[1]>1: # if by "up to 0.5" means not passing 0.5, should be 1, if need to be between 0 and 0.5, switch 1 to 0.5 here.
        noObstacle =  forward()
        if not noObstacle:
            turn_helper(90,True)
            noObstacle =  forward()
            if(noObstacle):
                find_object_helper(color)
                return
        distance_to_color = distance_helper(color)
        if distance_to_color is None:
            find_object_helper(color)
            return
        if distance_to_color[0] == 0:
            turn_helper(distance_to_color[0],True)
        else:
            if 360-distance_to_color[0]>180:
                turn_helper(distance_to_color[0],False)
            else:
                turn_helper(360-distance_to_color[0],True)
    
    print "I found the Object!"

def find_object():
    

    print "Please enter a color"
    color = raw_input()
    find_object_helper(color)

    
def talker():
    
    tasks = {"1": forward,
            "2": turn,
            "3": distance,
            "4": find_object}

    rospy.init_node('node1', anonymous=True)

    while not rospy.is_shutdown():
    
        print (print_msg)
        action = raw_input()
        print "The action number that was given is " + action
       
        if action in tasks:
            tasks[action]()
        else:
            print("Error: invalid task " + action)	

	

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
