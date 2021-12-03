#!/usr/bin/env python
# 
# VIPER: Vision Inference Processing (Edge) Endogenous Robot
# Andrew D'Amico
# MSDS 462 Computer Vision
# Northwestern University
# Copyright (c) 2021, Andrew D'Amico. All rights reserved.
# Licenced under BSD Licence.

###################################|####################################
############################ M O D U L E S #############################
############################# ROS MODULES ##############################
import rospy
from viper_toolkit import NameManager, ProcessTimer
from viper_toolkit import Parameter, ParameterManager

############################# STD MODULES ##############################
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from viper_toolkit import ProcessTimer, NameManager

######################### IMAGE SERVER MODULES #########################
#from model_server.srv import ImageRequest, ImageRequestResponse
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

######################### MODEL SERVER MODULES #########################
from model_server.msg import InferenceResults
from array import array

##### Pose Detection
#from pose_detector import PoseDetectionModel
from pose_detection_module import draw_poses

##### Scene Segmentation
#from scene_segmentation import SceneSegmentationModel
from scene_segmentation_module import segmentation_map_to_image

########################### OpenVino MODULES ###########################
#from openvino.inference_engine import IENetwork, IECore

######################### AUGMENTED VR MODULES #########################
from skimage import img_as_ubyte



class SegmentationDebugger(object):
    
    def __init__(self):
        rospy.init_node('SEG_Debugger', log_level = rospy.DEBUG)
        self.name = NameManager()
        #self.dynamic_updates()
        #self.pose_estimations = InferenceResults()
        self.setup_parameters()
        self.setup_ros()
        #self.setup_scene_segmentation()
        self.loop()

    def setup_parameters(self):
        # Adds all of our parameters to an updatable list. If ROS cannot
        # find the parameter, it will use the default provided.
        
        self.parameters = ParameterManager()
        
        self.parameters.add(Parameter("width", "/inland_ir_cam/width", default = 800, dynamic = True))
        self.parameters.add(Parameter("height", "/inland_ir_cam/height", 600, True))
        self.parameters.add(Parameter("updates", f"{self.name.name}/dynamic", True, True))
        self.parameters.add(Parameter("image_output", f"{self.name.name}/image_output", False, False))
        self.parameters.add(Parameter("alpha", rospy.search_param('alpha'), 0.3, True))
        self.parameters.add(Parameter("threshold", rospy.search_param('threshold'), 0.1, True))
        self.parameters.add(Parameter("segmentationmodel", f"{self.name.name}/segmentation", True, True))
        self.parameters.add(Parameter("posedetectionmodel", f"{self.name.name}/posedetection", True, True))
        self.parameters.add(Parameter("rate", f"{self.name.name}/rate", 25, True))

        
    def setup_ros(self):
        self.bridge = CvBridge()
        self.setup_camera()
        #self.setup_pose_detector()
        self.setup_scene_segmentation()
        self.setup_vr_publisher()
        
    def setup_vr_publisher(self):
        self.pub = rospy.Publisher(
            'SEG/debugger',
            Image,
            queue_size=1
            )
            
    def setup_camera(self):
        rospy.Subscriber("image_server/image",
            Image, 
            self.image_callback, 
            queue_size=1
            )
        rospy.loginfo("Camera Online")

        self.image = Image()
        self.new_image = True

        self.canvas = np.zeros((
                    self.parameters.height,
                    self.parameters.width, 
                    3
                    ), dtype = "uint8"
                    )
        self.segmentation_graph = self.canvas

    def setup_scene_segmentation(self):
        self.mask = InferenceResults()
        self.new_mask = False
        rospy.Subscriber(
            'model_output/scene_segmentation/segmentation_mask',
            InferenceResults,
            self.scene_segmentation_callback,
            queue_size=1
            )
        self.colormap = np.array([[68, 1, 84], [48, 103, 141], [53, 183, 120], [199, 216, 52]])
        rospy.loginfo("Scene Segmentation subscription active")

    def scene_segmentation_callback(self, msg):
        self.mask = msg
        self.new_mask = True
        #print ( "Incoming Shape:",self.mask.shape)
        rospy.loginfo("[SEG] New Masks Received")
   
    def image_callback(self, msg):
        self.image = msg
        self.new_image = True
        rospy.loginfo("[IMG] New Image Received")

    def decode_inference_results(self, results, model: str):
        
        name = model
        
        try:
            structure = results.structure
            rospy.logdebug(f'[{name}] Converting {type(results.structure)} to structure.')
            rospy.logdebug(f'[{name}] Structure: {structure}')
            print(name, structure)
        except:
            rospy.logerr(
            f"[{name}] Cannot convert {type(results.structure)} to structure"
            )
            return
        
        print ("------------------------------------------")
        
        try:
            inferences = np.asarray(results.inferences)
            rospy.logdebug(f"[{name}] Transformed shape: {inferences.shape}")
        except:
            rospy.logerr(f'[{name}] Cannot convert to numpy array')
            return
            
        print ("------------------------------------------")

        #try:
        inference_array = inferences.reshape(structure)
        rospy.logdebug(f'Wrangled Shape: {inference_array.shape}')
        rospy.logdebug(f'Blob Type: {type(inference_array)}')
        #except:
        #    rospy.logerr(f"[{name}] Cannot wrangle data blob")
        #    return

        return inference_array
            
    def update_segmentation_graph(self):
        '''
        Creates a properly sized segmentation graph equal to the original
        image size
        '''
        inference_array = self.decode_inference_results(
                results = self.mask, 
                model = "SEG"
                )
        
        #segmask = self.bridge.cv2_to_imgmsg(inference_array[0])
        
        #self.segpub.publish(segmask)
        
        mask = segmentation_map_to_image(inference_array, self.colormap)
        
        rospy.logdebug(f'dsize: ({self.image.width}, {self.image.height}')
        
        # This is the properly size segmentation graph
        self.segmentation_graph = cv2.resize(
                        mask, 
                        (self.image.width, self.image.height))
            
        return


    def combine_segmentation_image(self):

        alpha = 0.3
        #self.timer.lap("Image Overlay")

        self.canvas = cv2.addWeighted(
            self.segmentation_graph, 
            alpha, #self.parameters.alpha, 
            self.canvas, 
            1 - alpha, 
            0)

        return 


    def action_loop(self):

        if self.new_mask:
            # Saves a new segmentation mask to self.segmentation_graph
            self.update_segmentation_graph()
            self.new_mask = False
        
        
        # If we are outputting the original image with the mask
        if self.parameters.image_output: #self.parameters.image_output:
            # If there is a new image to format
            if self.new_image:
                try:
                    # Convert the camera image to ROS and save it as the canvas
                    self.canvas = self.bridge.imgmsg_to_cv2(
                        self.image, 
                        desired_encoding="bgr8"
                        )
                    # set the new image to false in case this node runs
                    # faster than the image server.
                    self.name_image = False
                except CvBridgeError as e:
                    # if this fails then give us the last good image.
                    rospy.logerr("[IMG] Error converting ROS msg to image.")
                    print(e)
            # if the segmentation model is turned on.
            if self.parameters.segmentationmodel:
                try:
                    # Combine the seg model and the 'self.canvas' layer.
                    self.combine_segmentation_image()
                except:
                    rospy.logerr("[SEG] Error proccessing scene")
        # Otherwise, if not outputting original image
        else:
            # But if segmentation is turned on.
            if self.parameters.segmentationmodel:
                #replace the self.canvas with our mask
                self.canvas = img_as_ubyte(self.segmentation_graph)
            else:
            # Otherwise just display a black image
                self.canvas = np.zeros((
                    self.height, 
                    self.width, 
                    3
                    ), dtype = "uint8"
                    )

        try:
            #Convert the canvas back into a ROS message and return the msg
            image_ros = self.bridge.cv2_to_imgmsg(self.canvas, 'bgr8')
            return image_ros
            
        except (CvBridgeError, TypeError, UnboundLocalError) as e:
            rospy.logerr("[IMG]Error converting to ROS msg")
            print (e)
            return self.image

    def loop(self):

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():  
            ros_image = self.action_loop()
            self.pub.publish(ros_image)
            rate.sleep()

if __name__ == '__main__':
    SegmentationDebugger()
