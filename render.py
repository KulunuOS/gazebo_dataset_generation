#%% Importing libraries
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

#%% Create the directories if they don't already exist
#rospy.signal_shutdown("done ")
if not os.path.exists('dataset/rgb'):
    os.makedirs('dataset/rgb')
if not os.path.exists('dataset/depth'):
    os.makedirs('dataset/depth')
if not os.path.exists('dataset/meta'):
    os.makedirs('dataset/meta')
if not os.path.exists('dataset/mask'):
    os.makedirs('dataset/mask')
if not os.path.exists('dataset/model_pointcloud'):
    os.makedirs('dataset/model_pointcloud') 

#%% Pass the models to be rendered as arguments via argument parser
parser = argparse.ArgumentParser(description = 'parse some parameters')
parser.add_argument("models", nargs='+', help="Enter the names of the models seperated by a space")
#args = parser.parse_args('bottom_casing left_gear'.split())
args = parser.parse_args()
n_models = len(args.models)
print('Generating data for '+str(n_models)+' selected models')

#%% Initialize ros
rospy.init_node('data_render_gazebo', anonymous = True)
rate = rospy.Rate(0.5)
bridge = CvBridge()
cam_info_msg = rospy.wait_for_message('kinect1/color/camera_info', CameraInfo, timeout = 2)


#%% Set camera pose in gazebo
def set_cam_state_gazebo(camPos, camTwist):
    # Set cam state in gazebo
    camstate = stateGZ('kinect_ros::link', camPos, camTwist, 'world' )
    print('Transforming camera to pose : '+str(sample_num))
    try:
       gzclient = rospy.ServiceProxy('gazebo/set_link_state', setStateGZ)
       resp = gzclient(camstate)
        
    except Exception as inst:
           print('Error in gazebo/set_link_state service request: ' + str(inst) )

#%% Calculate parameters
def calc_params(phi,theta,dist):
    theta_rad = np.deg2rad(theta)
    phi_rad = np.deg2rad(phi)
    X = dist*np.cos(phi_rad)*np.cos(theta_rad)
    Y = dist*np.cos(phi_rad)*np.sin(theta_rad)
    Z = np.abs(dist*np.sin(phi_rad)) + 0.84

    cam_euler = R.from_euler('xyz',[0,phi,theta+180], degrees=True)
    cam_quat = cam_euler.as_quat()
    
    camPos = Pose(position= Point(x=X, y=Y, z=Z), 
                  orientation= Quaternion(x=cam_quat[0], y=cam_quat[1] , z=cam_quat[2], w=cam_quat[3]))
    camTwist = Twist(linear= Vector3(x=0, y=0, z=0) , 
                     angular= Vector3(x=0, y=0, z=0))
    
    return camPos,camTwist, X, Y,Z, cam_euler 

 #%% Function to convert between Image types for depth images
def convert_types(img, orig_min, orig_max, tgt_min, tgt_max, tgt_type):

    #info = np.finfo(img.dtype) # Get the information of the incoming image type
    # normalize the data to 0 - 1
    img_out = img / (orig_max-orig_min)   # Normalize by input range
    img_out = (tgt_max - tgt_min) * img_out # Now scale by the output range
    img_out = img_out.astype(tgt_type)

    #cv2.imshow("Window", img)
    return img_out

#%% Avoid duplicates
def check_dup():
    rgb_duplicate = True                     
    while rgb_duplicate:
        print('Subscribing to rgb topics...')
        img_msg = rospy.wait_for_message('/kinect1/color/image_raw', Image, timeout = 3)
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if sample_num > 0:  
            #No point checking for the sample 0
            previous_im = cv2.imread('dataset/rgb/'+str(sample_num-1)+'.png', -1)
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
            previous_im = cv2.imread('dataset/depth/'+str(sample_num-1)+'.png', -1)
            depth_duplicate = abs(np.nanmean(cv_depthImage - previous_im))< 200  # Mean of all pixels shouldn't be this small if it's two different images
            print('depth diff: '+str(np.nanmean(cv_depthImage - previous_im)))# - previous_im)))
            print(depth_duplicate)
            if depth_duplicate:
                #Try setting state again. Sometimes gazebo trips out as well.
                set_cam_state_gazebo(camPos, camTwist)
        else:
            depth_duplicate = False
          
    return cv_image, cv_depthImage

#%% Get Camera Extrinsics
def get_camera_extrinsics(phi,theta, dist):
    
    _,_,X,Y,Z,_ = calc_params(phi,theta,dist)

    cam_euler = R.from_euler('xyz',[0,phi,theta+180], degrees=True)
    cam_world_R = cam_euler.as_matrix()
    cam_world_t = np.array([X,Y,Z]).reshape(3,1)
    cam_world_T = np.hstack((cam_world_R,cam_world_t))
    cam_world_T = np.vstack((cam_world_T, [0,0,0,1]))
    
    return cam_world_T

#%% get object states and save pose
def get_object_states(n_models):
    resp=[]
    #Get object state 
    try: 
        rospy.wait_for_service('gazebo/get_model_state')
        client = rospy.ServiceProxy('gazebo/get_model_state', getStateGZ)
        for i in range(n_models):
            #print(args.models[i])
            #print(client(args.models[i], 'world'))
            resp.append( client(args.models[i], 'world'))
    except Exception as inst:
        print('Error in gazebo/get_link_state service request: ' + str(inst) )
    return resp

#%% True Object pose in world frame obtained from Gazebo Service
def get_object2cam_pose(resp, n_models, cam_world_T):
    obj_cam_T = np.zeros((  4, 4, n_models )) # Transformation Mats for 10 object classes
    
    for i in range(0 , n_models):
        obj_pos = np.array([resp[i].pose.position.x, resp[i].pose.position.y, resp[i].pose.position.z]).reshape(3,1)
        obj_or = [resp[i].pose.orientation.x, resp[i].pose.orientation.y, resp[i].pose.orientation.z, resp[i].pose.orientation.w]
        obj_or = (R.from_quat(obj_or)).as_matrix()
        obj_world_T = np.concatenate((obj_or, obj_pos), axis = 1) 

        # Transformation from object2world to  object2cam for GT label poses
        #obj_cam_T = np.dot(obj_world_T, np.linalg.inv(cam_world_T) )
        obj_world_T = np.vstack(( obj_world_T, [0,0,0,1] ))    
        obj_cam_T[:, :, i] = np.dot( np.linalg.inv(cam_world_T), obj_world_T )#[:3,:]
    
    gt_dict = { 'poses':obj_cam_T[:3,:,:] } #returns [ R  T , i] 
    return  gt_dict, obj_cam_T

#%% Load the meshes of all objects convert them to point clouds
# combine and return the pointclouds of all meshes in a dictionary

def mesh2pcld(n_models):
    all_points = {}
    all_pclds  = {}

    for i in range (0,n_models):
        mesh = o3d.io.read_triangle_mesh('data_generation/models/'+str(args.models[i])+'/meshes/'+str(args.models[i])+'.STL')
        poisson_pcld = mesh.sample_points_poisson_disk(number_of_points=30000) 
        all_pclds[str(args.models[i])] = poisson_pcld
        o3d.io.write_point_cloud('dataset/model_pointcloud/'+str(args.models[i])+'.ply', poisson_pcld )
        all_points[str(args.models[i])] = np.asarray(poisson_pcld.points)#, dtype= np.float32)
        
    return all_points, all_pclds

#%% Function to fill empty spaces in point cloud project
def cv_im_fill(bin_im):
    im_floodfill = bin_im.copy()
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels smaller than the image.
    h, w = bin_im.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two images to get the foreground.
    im_out = bin_im | im_floodfill_inv
    return im_out

#%% Transform the pointclouds to binary mask
def pcl_2_binary_mask(obj_cam_T,n_models, all_points):
    
    #Projection Matrix / Camera instrinsics
    cam_P = np.array(cam_info_msg.P).reshape(3,4)
    cam_P = np.vstack((cam_P , [0,0,0,1]))
    bin_mask = np.zeros((cam_info_msg.height, cam_info_msg.width), dtype= np.uint8)
    mask_list = []
    pixels_list = []
    
    #Camera optical link 
    cam2optical = R.from_euler('zyx',[1.57, 0, 1.57])
    cam2optical = cam2optical.as_matrix()
    op2cam_T = np.hstack(( np.vstack(( cam2optical , [0,0,0] )) , np.array([[0],[0],[0],[1]]) ))
    
    
    for i in np.argsort(-obj_cam_T[2,3,:]):
        print(str(args.models[i]))
        print(i)
        # copy all  the points
        cloud_temp = copy.deepcopy(all_points[str(args.models[i])]).transpose().astype(np.float32)
        cloud_temp = np.vstack(( cloud_temp, np.ones((1,cloud_temp.shape[1])) )).astype(np.float32)
        
        # Then transform it into camera's coordinate system
        cloud_cam = np.dot(  obj_cam_T[:, :, i]  , cloud_temp).astype(np.float32)
        
        # transform from camera-link to camera optical link
        cloud_optical = np.dot(op2cam_T, cloud_cam).astype(np.float32)
        
        # perspective projection into image-plane
        x,y,z,w = np.dot( cam_P, cloud_optical ).astype(np.float32) #This is the projection step
        print(z)
        x = x / z
        y = y / z
        
        print(x)

        #clips out all the points projected out of image height and width
        clipping = np.logical_and( np.logical_and(x>=0, x<=640) , np.logical_and(y>=0, y<=480) )
        x = x[np.where(clipping)]
        y = y[np.where(clipping)]
        
        #print(np.shape(x))
        
        #Leave the background black
        pixels = np.vstack((x,y)).transpose()
        pixels = np.array(pixels, dtype=np.uint16)
        #print(np.shape(pixels))
        pixels_list.append([pixels])
        
        this_mask = np.zeros((cam_info_msg.height, cam_info_msg.width), dtype= np.uint8)
        
        for point in pixels:
            this_mask[point[1]-1, point[0]-1] = 255
        
        this_mask = cv_im_fill(this_mask)
        
        this_mask[this_mask.nonzero()] = 1.05*np.ceil(255*(i+1)/n_models)
        r,c = this_mask.nonzero()
        #print(np.unique(this_mask[r,c]))
        #mask_list.append(this_mask)
        
        bin_mask[this_mask.nonzero()] = 0
        bin_mask += this_mask
           
    return bin_mask

#%% Main program

# convert the meshes to pointclouds 
print('Generating the pointclouds for all models. hold on , This could take few minutes ...')
all_points, all_pclds = mesh2pcld(n_models)

sample_num = 0
for dist in np.arange(0.15,0.5,0.125):
    for phi in range(35,90,15):
        for theta in range(0, 360, 15):
            camPos,camTwist,_,_,_,_ =  calc_params(phi,theta,dist)
            set_cam_state_gazebo(camPos, camTwist)
            
            
            while not rospy.is_shutdown():
                print('Subscribing to camera topics...')
                try:
                    cv_image, cv_depthImage = check_dup()
                    break  
                except ROSException as e:
                        print('Timeout occured in subscribing.Trying again...')
                        continue
            
            cv_depthImage = convert_types(cv_depthImage,0,3, 0,65535, np.uint16) ## 0 - 3m is the input range of kinect depth
            print('Writing Images')
            cv2.imwrite('dataset/rgb/'+str(sample_num)+'.png', cv_image)
            cv2.imwrite('dataset/depth/'+str(sample_num)+'.png',cv_depthImage)
                    
            try:
                resp = get_object_states(n_models)
            except Exception as inst:
                     print('Error in gazebo/get_link_state service request: ' + str(inst) )
                
            cam_world_T = get_camera_extrinsics(phi,theta,dist)
            gt_dict,obj_cam_T = get_object2cam_pose(resp, n_models,cam_world_T)
            
            sio.savemat('dataset/meta/'+str(sample_num)+'-meta.mat',gt_dict)
        
            bin_mask= pcl_2_binary_mask(obj_cam_T, n_models, all_points)
            
            cv2.imwrite('dataset/mask/'+str(sample_num)+'.png',bin_mask)
            
            sample_num += 1