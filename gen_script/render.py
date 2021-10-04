#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import numpy as np
import open3d as o3d
import ros_numpy
import rospy
import tf
import yaml
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import LinkState as stateGZ
from gazebo_msgs.srv import GetModelState as getStateGZ
from gazebo_msgs.srv import SetLinkState as setStateGZ
from geometry_msgs.msg import (Point, Pose, PoseArray, PoseStamped, Quaternion,
                               Twist, Vector3)
from rospy.exceptions import ROSException
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import copy
import time

import cv2
import pcl
import scipy.io as sio

rospy.init_node('data_render_gazebo')
rate = rospy.Rate(0.5)
bridge = CvBridge()
br = tf.TransformBroadcaster()

## Load all the .ply files here for projecting onto the binary-label image ##

all_meshes = {}
for i in range(0,8):
    #this_mesh = pcl.load('/home/ahmad3/PVN3D/pvn3d/datasets/openDR/openDR_dataset/models/obj_'+str(i+1)+'.ply')
    this_mesh = o3d.io.read_point_cloud('/home/ahmad3/PVN3D/pvn3d/datasets/CrankSlider/CrankSlider_dataset/models/Scale0.02/obj_'+str(i+1)+'.ply')
    all_meshes['obj_'+str(i+1)] = np.asarray(this_mesh.points)#, dtype= np.float32)
#print(all_meshes)

cam2optical = R.from_euler('zyx',[1.57, 0, 1.57])
cam2optical = cam2optical.as_dcm()
op2cam_T = np.hstack(( np.vstack(( cam2optical , [0,0,0] )) , np.array([[0],[0],[0],[1]]) ))

def set_cam_state_gazebo(camPos, camTwist):
    # Set cam state in gazebo
    camstate = stateGZ('kinect_ros::link', camPos, camTwist, 'world' )
    print('Transforming camera to pose '+str(sample_num))
    try:
       gzclient = rospy.ServiceProxy('gazebo/set_link_state', setStateGZ)
       resp = gzclient(camstate)

    except Exception as inst:
           print('Error in gazebo/set_link_state service request: ' + str(inst) )



def convert_types(img, orig_min, orig_max, tgt_min, tgt_max, tgt_type):

    #info = np.finfo(img.dtype) # Get the information of the incoming image type
    # normalize the data to 0 - 1
    img_out = img / (orig_max-orig_min)   # Normalize by input range
    img_out = (tgt_max - tgt_min) * img_out # Now scale by the output range
    img_out = img_out.astype(tgt_type)

    #cv2.imshow("Window", img)
    return img_out


def cv_im_fill(bin_im):
    ### https://www.learnopencv.com/filling-holes-in-an-image-using-opencv-python-c/ ###

    # Copy the binary image.
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


sample_num = 3301   # set this to 0 at first r
gt_yaml_dict = {}
cam_info_msg = rospy.wait_for_message('kinect1/color/camera_info', CameraInfo, timeout = 2)

### Uniformly Sample poses in upper hemisphere around the object ###
## http://corysimon.github.io/articles/uniformdistn-on-sphere/ ##

## Distance(m) increments for covering different scales around the object
#0.65,1.05, 0.125
for dist in np.arange(0.55,1.05,0.125):

    ## Pitch(deg) increments for covering different views of object  along y-axis
    # This is along axis of object to cover all sides of the object
    # 35, 90, 10	
    for phi in range(35,90,15):


            ## Yaw(deg) increments for covering different views of object  along z-axis
            # This is along axis of object to cover all sides of the object
            # 0, 360, 15
            for theta in range(0, 360, 15):

                theta_rad = np.deg2rad(theta)
                phi_rad = np.deg2rad(phi)
                X = dist*np.cos(phi_rad)*np.cos(theta_rad)
                Y = dist*np.cos(phi_rad)*np.sin(theta_rad)
                Z = np.abs(dist*np.sin(phi_rad))

                cam_euler = R.from_euler('xyz',[0,phi,theta+180], degrees=True)
                cam_quat = cam_euler.as_quat()

                camPos = Pose(position= Point(x=X, y=Y, z=Z), orientation= Quaternion(x=cam_quat[0], y=cam_quat[1] , z=cam_quat[2], w=cam_quat[3]))
                camTwist = Twist(linear= Vector3(x=0, y=0, z=0) , angular= Vector3(x=0, y=0, z=0))

                set_cam_state_gazebo(camPos, camTwist)
                rospy.sleep(0.15)

                # Subscribe to rgb and depth images
                '''
                try:
                    usrIn = input('Press Enter to continue to Subscribing when the camera has properly entered the new pose...')
                except:
                    usrIn = None'''

                while not rospy.is_shutdown():
                    print('Subscribing to camera topics...')


                    try:

                        rgb_duplicate = True

                        ''' Following is a quick-fix for subscribing problems in ROS, the image topics...
                        sometimes don't update as quickly as we want them to and we end up with a few...
                        duplicate images creeping somewhere in the final data. This hack just uses the...
                        diff with previous image to check similarity and proceeds only when the topics
                        actually seem updated. There must be a better way to do this. '''

                        while rgb_duplicate:

                            print('Subscribing to rgb topics...')
                            img_msg = rospy.wait_for_message('/kinect1/color/image_raw', Image, timeout = 3)
                            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
                            #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

                            if sample_num > 0:  #No point checking for the sample 0
                                previous_im = cv2.imread('/home/ahmad3/PVN3D/pvn3d/datasets/CrankSlider/CrankSlider_dataset/rgb/'+str(sample_num-1)+'.png', -1)
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
                                previous_im = cv2.imread('/home/ahmad3/PVN3D/pvn3d/datasets/CrankSlider/CrankSlider_dataset/depth/'+str(sample_num-1)+'.png', -1)
                                depth_duplicate = abs(np.nanmean(cv_depthImage - previous_im))< 200  # Mean of all pixels shouldn't be this small if it's two different images
                                print('depth diff: '+str(np.nanmean(cv_depthImage - previous_im)))# - previous_im)))
                                print(depth_duplicate)
                                if depth_duplicate:
                                    #Try setting state again. Sometimes gazebo trips out as well.
                                    set_cam_state_gazebo(camPos, camTwist)
                            else:
                                depth_duplicate = False


                        #cloud_msg = rospy.wait_for_message('kinect1/depth/points', PointCloud2, timeout = 3)'''
                        break

                    except ROSException as e:
                        print('Timeout occured in subscribing.Trying again...')
                        #rospy.spin()
                        continue

                # Get object state in Gazebo
                #resp = np.zeros((10))
                resp = []

                try:
                     rospy.wait_for_service('gazebo/get_model_state')
                     client = rospy.ServiceProxy('gazebo/get_model_state', getStateGZ)
                     resp.append( client('Bearing', 'world'))
                     resp.append( client('Cam', 'world'))
                     resp.append( client('Connecting-rod-Lower', 'world'))
                     resp.append( client('Connecting-rod-Upper', 'world'))
                     resp.append( client('Piston', 'world')#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import numpy as np
import open3d as o3d
import ros_numpy
import rospy
import tf
import yaml
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import LinkState as stateGZ
from gazebo_msgs.srv import GetModelState as getStateGZ
from gazebo_msgs.srv import SetLinkState as setStateGZ
from geometry_msgs.msg import (Point, Pose, PoseArray, PoseStamped, Quaternion,
                               Twist, Vector3)
from rospy.exceptions import ROSException
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import copy
import time

import cv2
import pcl
import scipy.io as sio
)
                     resp.append( client('Piston-Pin', 'world'))
                     resp.append( client('Seal1', 'world'))
                     resp.append( client('Seal2', 'world'))

                except Exception as inst:
                     print('Error in gazebo/get_link_state service request: ' + str(inst) )

                # Camera Extrinsics
                cam_world_R = cam_euler.as_dcm()
                cam_world_t = np.array([X,Y,Z]).reshape(3,1)
                cam_world_T = np.hstack((cam_world_R,cam_world_t))
                cam_world_T = np.vstack((cam_world_T, [0,0,0,1]))

                # True Object pose in world frame obtained from Gazebo Service
                obj_cam_T = np.zeros((  4, 4, 8 )) # Transformation Mats for 10 object classes
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

import numpy as np
import open3d as o3d
import ros_numpy
import rospy
import tf
import yaml
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import LinkState as stateGZ
from gazebo_msgs.srv import GetModelState as getStateGZ
from gazebo_msgs.srv import SetLinkState as setStateGZ
from geometry_msgs.msg import (Point, Pose, PoseArray, PoseStamped, Quaternion,
                               Twist, Vector3)
from rospy.exceptions import ROSException
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import copy
import time

import cv2
import pcl
import scipy.io as sio

                for i in range(0 , 8):
                    obj_pos = np.array([resp[i].pose.position.x, resp[i].pose.position.y, resp[i].pose.position.z]).reshape(3,1)
                    obj_or = [resp[i].pose.orientation.x, resp[i].pose.orientation.y, resp[i].pose.orientation.z, resp[i].pose.orientation.w]
                    obj_or = (R.from_quat(obj_or)).as_dcm()
                    obj_world_T = np.concatenate((obj_or, obj_pos), axis = 1)


                    # Transformation from object2world to  object2cam for GT label poses
                    #obj_cam_T = np.dot(obj_world_T, np.linalg.inv(cam_world_T) )
                    obj_world_T = np.vstack(( obj_world_T, [0,0,0,1] ))
                    obj_cam_T[:, :, i] = np.dot( np.linalg.inv(cam_world_T), obj_world_T )#[:3,:]
                gt_dict = { 'poses':obj_cam_T[:3,:,:] }
                sio.savemat('/home/ahmad3/PVN3D/pvn3d/datasets/CrankSlider/CrankSlider_dataset/meta/'+str(sample_num)+'-meta.mat',gt_dict)

                # Kinect in gazebo has it's depth registered to color camera so intrinsics...
                # of color cam can be used for projection
                cam_P = np.array(cam_info_msg.P).reshape(3,4)  #Projection matrix or the intrinsics
                cam_P = np.vstack((cam_P , [0,0,0,1]))
                obj_mask = np.zeros((cam_info_msg.height, cam_info_msg.width), dtype= np.uint8)

                # 3D to 2D Projection based on camera extrinsics & intrinsics
                # The scanned_cloud/mesh is projected onto image with the current...
                # camera pose in world, in order to obtain the pixels that represent
                # the object in the current view i.e; binary mask

                ## https://stackoverflow.com/questions/724219/how-to-convert-a-3d-point-into-2d-perspective-projection
                ## https://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points
                print('Transforming meshes to binary mask...')


                for i in np.argsort(-obj_cam_T[2,3,:]):
                    ## This sorting is from the object farthest from the camera to the nearest...
                    ## since the meshes nearer to the camera are going to occlude the meshes...
                    ## further away in a cluttered scene

                    start = time.time()*1000.0
                    cloud_temp = copy.deepcopy(all_meshes['obj_'+str(i+1)]).transpose().astype(np.float32) # We don't want to mess with the original copy
                    #cloud_temp = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_msg).transpose()

                    cloud_temp = np.vstack(( cloud_temp, np.ones((1,cloud_temp.shape[1])) )).astype(np.float32)
                    #print cloud_temp.shape

                    # Transform the mesh first into true object2world coordinate system
                    # i.e; we want the mesh to be projected where the object actually is...
                    # Skip this if we're sure that the target object is at (0,0,0,0,0,0) in gazebo
                    # It's also assumed that the Mesh was recorded on (0,0,0,0,0,0) pose...
                    # w.r.t. the same world frame being used for rendering, if not than
                    # this mesh is not useful, since it cannot be directly transformed
                    # to the current frame of reference
                    #cloud_world = np.dot(obj_world_T, cloud_temp )#.transpose()
                    #print(cloud_world.shape)
                    # Then transform it into camera's coordinate system i.e; This puts mesh in the right viewpoint
                    cloud_cam = np.dot(  obj_cam_T[:, :, i]  , cloud_temp).astype(np.float32)
                    #cloud_cam = np.dot( np.linalg.inv(cam_world_T), cloud_world)

                    # This step transforms from camera_link to camera_optical_link, they are the mirror of each
                    # other. The projection is inverted if the scans from camera_link are directly used.

                    #optical2cam = np.linalg.inv(cam2optical)


                    cloud_optical = np.dot(op2cam_T, cloud_cam).astype(np.float32)

                    # Last step is perspective projection into image-plane

                    # This clipping method is from the stackoverflow link above.
                    # All the parameters required to form this clip matrix are
                    # already on ros camera_info.P matrix. So this is not used
                    # This is general for all pinhole camera models if intrinsics
                    # are is not readily available on ROS topics

                    #far = 3 #far clipping distance
                    #near = 0.01 #near clipping distance
                    #fov = 1.047198
                    #f= 1/np.tan(fov/2)
                    #aspectRatio = 640/480
                    #clip_Mat = np.array([[f*aspectRatio,             0, 0, 0],
                    #                     [0, f,                         0, 0],
                    #                     [0, 0, (far + near)/(far - near), 1],
                    #                     [0, 0, (2*far*near)/(near - far), 0]])


                    x,y,z,w = np.dot( cam_P, cloud_optical ).astype(np.float32) #This is the projection step
                    x = x / z
                    y = y / z

                    #print ''
                    #print x.max()
                    #print x.min()
                    #print y.max()
                    #print y.min()
                    #print ''


                    # This step clips out all the points projected out of image height and width
                    clipping = np.logical_and( np.logical_and(x>=0, x<=640) , np.logical_and(y>=0, y<=480) )
                    x = x[np.where(clipping)]
                    y = y[np.where(clipping)]

                    pixels = np.vstack((x,y)).transpose()  # shape it to N x 2 for looping

                    pixels = np.array(pixels, dtype=np.uint16)

                    this_mask = np.zeros((cam_info_msg.height, cam_info_msg.width), dtype= np.uint8)

                    for point in pixels:   # Loops pixel.shape[0] times
                        #print point
                        this_mask[point[1]-1, point[0]-1] = 255 #1.05*np.ceil(255*(i+1)/10)   # Points with projected x,y coordinates are mapped to unique grey labe
                    # Fill gaps in the binary image - Gaps are inevitable when backprojecting pointclouds
                    this_mask = cv_im_fill(this_mask)

                    #Consecutively add each mask to the original mask to get a combined mask of all classes
                    this_mask[this_mask.nonzero()] = 1.05*np.ceil(255*(i+1)/8)
                    obj_mask[this_mask.nonzero()] = 0
                    obj_mask += this_mask
                    print('Total rendering time: '+str(time.time()*1000.0-start)+' ms per class')


                # Use depthimage for mask generation

                #obj_mask = np.zeros((cv_depthImage.shape))
                #obj_mask[np.where(cv_depthImage>1e-8)] = 255


                # Check the correctness of GT mask by taking difference from RGB Image
                #cv_img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                #_,bin_img = cv2.threshold(cv_img_gray,100,255,cv2.THRESH_BINARY)
                #diff_img = np.absolute(bin_img+obj_mask)


                # Convert from float32 to uint16
                cv_depthImage = convert_types(cv_depthImage,0,3, 0,65535, np.uint16) ## 0 - 3m is the input range of kinect depth
                #cv_depthImage[np.isnan(cv_depthImage)] = 0'''

                # Save the images
                print('Writing Images')
                cv2.imwrite('/home/ahmad3/PVN3D/pvn3d/datasets/CrankSlider/CrankSlider_dataset/rgb/'+str(sample_num)+'.png', cv_image)
                cv2.imwrite('/home/ahmad3/PVN3D/pvn3d/datasets/CrankSlider/CrankSlider_dataset/depth/'+str(sample_num)+'.png',cv_depthImage)
                cv2.imwrite('/home/ahmad3/PVN3D/pvn3d/datasets/CrankSlider/CrankSlider_dataset/mask/'+str(sample_num)+'.png',obj_mask)
                #cv2.imwrite('/home/ahmad3/PVN3D/pvn3d/datasets/linemod/Linemod_preprocessed/data/16/diff/'+str(sample_num)+'.png',diff_img)
                #cv2.imwrite('/home/ahmad3/PVN3D/pvn3d/datasets/linemod/Linemod_preprocessed/data/16/thresh/'+str(sample_num)+'.png',bin_img)

                # Publish transforms to check poses in rviz

                br.sendTransform(( X, Y, Z), (cam_quat[0], cam_quat[1], cam_quat[2], cam_quat[3]),rospy.Time.now(), 'camera_link',"world")
                for i in range(0,8):
                    obj_cam_or = R.from_dcm(obj_cam_T[0:3,0:3,i])
                    o2c_or = obj_cam_or.as_quat()
                    br.sendTransform((obj_cam_T[0,3,i], obj_cam_T[1,3,i] , obj_cam_T[2,3,i]),(o2c_or[0], o2c_or[1], o2c_or[2], o2c_or[3]),rospy.Time.now(), 'object'+str(i),"camera_link")
               
                #print str(cv_image.dtype)+' '+ str(cv_image.shape) + ' ' + str(cv_image.max())

                #print str(cv_depthImage.dtype)+' '+ str(cv_depthImage.shape) + ' ' + str(np.nanmax(cv_depthImage))

                sample_num += 1
                #rospy.spin()

