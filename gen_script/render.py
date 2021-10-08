# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import sys
import numpy as np
import open3d as o3d
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import LinkState as stateGZ
from gazebo_msgs.srv import GetModelState as getStateGZ
from gazebo_msgs.srv import SetLinkState as setStateGZ
from geometry_msgs.msg import (Point, Pose, PoseArray, PoseStamped, Quaternion,
                               Twist, Vector3)
from rospy.exceptions import ROSException
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

import copy
import time
import cv2
import scipy.io as sio
import os
import argparse

#%% Argument parser
parser = argparse.ArgumentParser(description = 'parse some parameters')
parser.add_argument("models", nargs='+', help="Enter the names of the models seperated by a space")
args = parser.parse_args()
#args = parser.parse_args('left_gear'.split())

n_models = len(args.models)


#%% Create the directories
rospy.init_node('data_render_gazebo')
rate = rospy.Rate(0.5)
bridge = CvBridge()
#rospy.signal_shutdown("done ")
if not os.path.exists('rgb'):
    os.makedirs('rgb')
if not os.path.exists('depth'):
    os.makedirs('depth')
if not os.path.exists('meta'):
    os.makedirs('meta')    
    
#%% Function to set camera position and pose
def set_cam_state_gazebo(camPos, camTwist):
    # Set cam state in gazebo
    camstate = stateGZ('kinect_ros::link', camPos, camTwist, 'world' )
    print('Transforming camera to pose '+str(sample_num))
    try:
       gzclient = rospy.ServiceProxy('gazebo/set_link_state', setStateGZ)
       resp = gzclient(camstate)

    except Exception as inst:
           print('Error in gazebo/set_link_state service request: ' + str(inst) )
           


#%% Function to convert between Image types
def convert_types(img, orig_min, orig_max, tgt_min, tgt_max, tgt_type):

    #info = np.finfo(img.dtype) # Get the information of the incoming image type
    # normalize the data to 0 - 1
    img_out = img / (orig_max-orig_min)   # Normalize by input range
    img_out = (tgt_max - tgt_min) * img_out # Now scale by the output range
    img_out = img_out.astype(tgt_type)

    #cv2.imshow("Window", img)
    return img_out

#%% Check for RGB,Depth duplicate Images
def check_dup():
    rgb_duplicate = True                     
    while rgb_duplicate:
        print('Subscribing to rgb topics...')
        img_msg = rospy.wait_for_message('/kinect1/color/image_raw', Image, timeout = 3)
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if sample_num > 0:  
            #No point checking for the sample 0
            previous_im = cv2.imread('rgb/'+str(sample_num-1)+'.png', -1)
            rgb_duplicate = abs(np.mean(cv_image - previous_im)) < 2 # Mean of all pixels shouldn't be this small if it's two different images
            print('rgb diff: '+str(np.mean(cv_image - previous_im)))
            print(rgb_duplicate)
            if rgb_duplicate:
                #Try setting state again. Sometimes gazebo trips out as well.
                set_cam_state_gazebo(camPos, camTwist)

        else:
            rgb_duplicate = False

    depth_duplicate = True 
    while depth_duplicate:
        print('Subscribing to depth topics...')
        depthImg_msg = rospy.wait_for_message('/kinect1/depth/image_raw', Image, timeout = 3 )
        cv_depthImage = bridge.imgmsg_to_cv2(depthImg_msg, desired_encoding='passthrough')
        if sample_num > 0:
            previous_im = cv2.imread('depth/'+str(sample_num-1)+'.png', -1)
            depth_duplicate = abs(np.nanmean(cv_depthImage - previous_im))< 200  # Mean of all pixels shouldn't be this small if it's two different images
            print('depth diff: '+str(np.nanmean(cv_depthImage - previous_im)))# - previous_im)))
            print(depth_duplicate)
            if depth_duplicate:
                #Try setting state again. Sometimes gazebo trips out as well.
                set_cam_state_gazebo(camPos, camTwist)
        else:
            depth_duplicate = False
          
    return cv_image, cv_depthImage
    
#%% Main program
sample_num = 0

for dist in np.arange(0.15,0.5,0.125):
    for phi in range(35,90,15):
        for theta in range(0, 360, 15):
            
            theta_rad = np.deg2rad(theta)
            phi_rad = np.deg2rad(phi)
            X = dist*np.cos(phi_rad)*np.cos(theta_rad)
            Y = dist*np.cos(phi_rad)*np.sin(theta_rad)
            Z = np.abs(dist*np.sin(phi_rad)) + 0.84

            cam_euler = R.from_euler('xyz',[0,phi,theta+180], degrees=True)
            cam_quat = cam_euler.as_quat()

            camPos = Pose(position= Point(x=X, y=Y, z=Z), orientation= Quaternion(x=cam_quat[0], y=cam_quat[1] , z=cam_quat[2], w=cam_quat[3]))
            camTwist = Twist(linear= Vector3(x=0, y=0, z=0) , angular= Vector3(x=0, y=0, z=0))

            set_cam_state_gazebo(camPos, camTwist)
            rospy.sleep(0.15)
            
            while not rospy.is_shutdown():
                print('Subscribing to camera topics...')
                
                try:
                    cv_image, cv_depthImage = check_dup()
                    break  
                except ROSException as e:
                        print('Timeout occured in subscribing.Trying again...')
                        continue
                    
            # Convert from float32 to uint16
            cv_depthImage = convert_types(cv_depthImage,0,3, 0,65535, np.uint16) ## 0 - 3m is the input range of kinect depth
            print('Writing Images')
            cv2.imwrite('rgb/'+str(sample_num)+'.png', cv_image)
            cv2.imwrite('depth/'+str(sample_num)+'.png',cv_depthImage)
            sample_num += 1
            
            resp=[]
            #Get object state 
            try: 
                rospy.wait_for_service('gazebo/get_model_state')
                client = rospy.ServiceProxy('gazebo/get_model_state', getStateGZ)
                for i in range(len(args.models)):
                    resp.append( client(args.models[i], 'world'))
            except Exception as inst:
                     print('Error in gazebo/get_link_state service request: ' + str(inst) )
        
             # Camera Extrinsics
            cam_world_R = cam_euler.as_dcm()
            cam_world_t = np.array([X,Y,Z]).reshape(3,1)
            cam_world_T = np.hstack((cam_world_R,cam_world_t))
            cam_world_T = np.vstack((cam_world_T, [0,0,0,1]))   
             
            # True Object pose in world frame obtained from Gazebo Service
            obj_cam_T = np.zeros((  4, 4, n_models )) # Transformation Mats for 10 object classes
            for i in range(0 , n_models):
                obj_pos = np.array([resp[i].pose.position.x, resp[i].pose.position.y, resp[i].pose.position.z]).reshape(3,1)
                obj_or = [resp[i].pose.orientation.x, resp[i].pose.orientation.y, resp[i].pose.orientation.z, resp[i].pose.orientation.w]
                obj_or = (R.from_quat(obj_or)).as_dcm()
                obj_world_T = np.concatenate((obj_or, obj_pos), axis = 1) 
                
                # Transformation from object2world to  object2cam for GT label poses
                #obj_cam_T = np.dot(obj_world_T, np.linalg.inv(cam_world_T) )
                obj_world_T = np.vstack(( obj_world_T, [0,0,0,1] ))
                obj_cam_T[:, :, i] = np.dot( np.linalg.inv(cam_world_T), obj_world_T )#[:3,:]
            gt_dict = { 'poses':obj_cam_T[:3,:,:] }
            sio.savemat('meta/'+str(sample_num)+'-meta.mat',gt_dict)
            
            
