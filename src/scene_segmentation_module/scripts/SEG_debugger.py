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




class SegmentationDebugger(object):
    
    def __init__(self):
        #self.name = NameManager()
        #self.dynamic_updates()
        #self.pose_estimations = InferenceResults()
        #self.setup_parameters()
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
        self.parameters.add(Parameter("image_output", f"{self.name.name}/image_output", True, True))
        self.parameters.add(Parameter("alpha", rospy.search_param('alpha'), 0.3, True))
        self.parameters.add(Parameter("threshold", rospy.search_param('threshold'), 0.1, True))
        self.parameters.add(Parameter("segmentationmodel", f"{self.name.name}/segmentation", True, True))
        self.parameters.add(Parameter("posedetectionmodel", f"{self.name.name}/posedetection", True, True))
        self.parameters.add(Parameter("rate", f"{self.name.name}/rate", 25, True))
        
    def setup_ros(self):
        rospy.init_node('SEG_Debugger', log_level = rospy.DEBUG)
        self.bridge = CvBridge()
        self.setup_camera()
        self.setup_pose_detector()
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

    def setup_scene_segmentation(self):
        self.mask = InferenceResults()
        self.new_mask = True
        rospy.Subscriber(
            'model_output/scene_segmentation/segmentation_mask',
            InferenceResults,
            self.scene_segmentation_callback,
            queue_size=1
            )
        self.mask = InferenceResults()
        self.colormap = np.array([[68, 1, 84], [48, 103, 141], [53, 183, 120], [199, 216, 52]])
        rospy.loginfo("Scene Segmentation subscription active")

    def scene_segmentation_callback(self, msg):
        self.mask = msg
        self.new_mask = True
        print (self.mask)
        rospy.loginfo("[SEG] New Masks Received")
   
    def image_callback(self, msg):
        self.image = msg
        self.new_image = True
        rospy.loginfo("[IMG] New Image Received")

    def decode_inference_results(self, results, model: str = "SEG"):

        try:
            structure = results.structure
            print(name, structure)
        except:
            rospy.logerr(
            f"[{name}] Cannot convert {type(self.results.structure)} to structure"
            )
            return
        print ("------------------------------------------")
        try:
            inferences = np.asarray(results.inferences)
            print(inferences)
            rospy.loginfo(f"[{name}] Transformed shape: {inferences.shape}")
        except:
            rospy.logerr(f'[{name}] Cannot convert to numpy array')
            return
            
        try:
            inference_array = inferences.reshape(results.structure)
            print(inference_array)
        except:
            rospy.logerr(f"[{name}] Cannot wrangle data blob")
            return

        return inference_array
            
    def update_segmentation_graph(self):

        inference_array = self.decode_inference_results(
                self.mask, 
                model="SEG"
                )
                
        mask = segmentation_map_to_image(inference_array, self.colormap)

        self.segmentation_graph = cv2.resize(
            inference_array, 
            (self.image.width, self.image.height))
            
        return


    def combine_segmentation_image(self):

        alpha = 0.3
        self.timer.lap("Image Overlay")

        self.canvas = cv2.addWeighted(
            self.segmentation_graph, 
            alpha, #self.parameters.alpha, 
            self.img, 
            1 - alpha, 
            0)

        return 


    def action_loop(self):

        if self.new_mask:
            self.update_segmentation_graph()
            self.new_mask = False
        
        if self.new_graph:
            self.calculate_pose_graph()
            self.new_graph = False
        
        image_output = True
        if image_output: #self.parameters.image_output:
            if self.new_image:
                try:
                    # Convert the camera image to ROS
                    self.img = self.bridge.imgmsg_to_cv2(
                        self.image, 
                        desired_encoding="bgr8"
                        )
                    self.new_graph = False
                except CvBridgeError as e:
                    # if this fails then give us the last good image.
                    rospy.logerr("[IMG] Error converting ROS msg to image.")
                    print(e)
                    
            if temp: #self.parameters.segmentationmodel:
                try:
                    # We can now combine them on the 'self.canvas' layer.
                    self.combine_segmentation_image()
                except:
                    rospy.logerr("[SEG] Error proccessing scene")
        else:
            if self.parameters.segmentation_model:

                self.canvas = self.segmentation_graph
            else:
                self.canvas = np.zeros((
                    self.height, 
                    self.width, 
                    3
                    ), dtype = "uint8"
                    )
            

        try:
            self.draw_graph()
        except:
            rospy.logerr("[POS] Error drawing pose detection graph")

        try:
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
