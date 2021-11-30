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




class AugmentedVRModule(object):
    
    def __init__(self):
        self.name = NameManager()
        #self.dynamic_updates()
        self.pose_estimations = InferenceResults()
        self.setup_parameters()
        self.setup_ros()
        self.setup_scene_segmentation()
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
        rospy.init_node('Augmented_VR', log_level = rospy.DEBUG)
        self.bridge = CvBridge()
        self.setup_camera()
        self.setup_pose_detector()
        self.setup_scene_segmentation()
        self.setup_vr_publisher()
        
    def setup_vr_publisher(self):
        self.pub = rospy.Publisher(
            'Augmented_VR/stream',
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

    def setup_pose_detector(self):
        rospy.Subscriber(
            'model_output/pose_articulator/human_graph_nodes',
            InferenceResults,
            self.pose_estimation_callback,
            queue_size=1
            )
        self.pose_estimations = InferenceResults()
        self.new_graph = True
        self.graph = []
        rospy.loginfo("Pose detector subscription active")

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

    def pose_estimation_callback(self, msg):
        self.pose_estimations = msg
        self.new_graph = True
        print (self.pose_estimations)
        rospy.loginfo("[POS] New Coordinates Received")

    def scene_segmentation_callback(self, msg):
        self.mask = msg
        self.new_mask = True
        print (self.mask)
        rospy.loginfo("[SEG] New Masks Received")
   
    def image_callback(self, msg):
        self.image = msg
        self.new_image = True
        rospy.loginfo("[IMG] New Image Received")

    def decode_inference_results(self, results, model: str = None):
        if model == None:
            name = self.name.abv
        else:
            name = model
        self.timer.lap("Decode Inference")
        rospy.logdebug(f"Decoding {name}")
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
            
        #self.timer.time("Decode Inference", name=name)
        
        return inference_array
            
    def update_segmentation_graph(self):
    # This function tasks a message file (self.mask) and decodes it into
    # the original array, outputing the "segmentation graph". Note that
    # this output can be used on its own as an image.
        print (self.mask)
        #self.timer.lap("Segmentation Process")
        #try:
        inference_array = self.decode_inference_results(
                self.mask, 
                model="SEG"
                )
        #rospy.loginfo(f'inference_array is: {type(inference_array)} {inference_array.shape}')
        #except:
        #    rospy.logerr(f"[SEG] Cannot decode results")
        #    return
        
        #try:
            #self.timer.lap("Segmentation Model")
            #rospy.logdebug(f"[{self.name.abv}] Conversion Shape: %s",inference_array.shape)
        print(type(inference_array))
        mask = segmentation_map_to_image(inference_array, self.colormap)
            #rospy.logdebug(f"[{self.name.abv}] Resizing Mask")
        self.segmentation_graph = cv2.resize(inference_array, (self.image.width, self.image.height))
        self.logdebug(f'seg graph shape {self.segmentation_graph.shape}')
            #rospy.logdebug(f"[{self.name.abv}] Masking Image")
            #self.timer.time("Segmentation Model", name="SEG")
            #self.timer.time("Segmentation Process", name="SEG")
        return
        #except:
        #    rospy.logerr(f"[SEG] Cannot resize mask at point A")
            #self.timer.time("Segmentation Model", name="SEG")
            #self.timer.time("Segmentation Process", name="SEG")
        #    return

    def combine_segmentation_image(self):
        # This function combines the current segmentation_graph 
        # and the current converted image, and saves it as the 
        # current output (canvas)
        alpha = 0.3
        self.timer.lap("Image Overlay")
        #try:
        self.canvas = cv2.addWeighted(
            self.segmentation_graph, 
            alpha, #self.parameters.alpha, 
            self.img, 
            1 - alpha, 
            0)
        #except:
        #    rospy.logerr(f"[SEG] Cannot combine mask and image at B")
            
        # Regardless of success we will update our timer.
        #self.timer.time("Image Overlay", name = "SEG")
        return 

    def calculate_pose_graph(self):
        # This process takes the current pose estimation msg
        # (self.pose_estimates) and converts them to the 
        # pose graph nodes and edges. It then saves these
        # to the current nodes and edges (self.graph)

        # Convert the message arrays
        #self.timer.lap("Graph Nodes Edges")
        rospy.logdebug("Decoding Pose Graph Results")
        #try:
        self.graph = self.decode_inference_results(
            self.pose_estimations, 
            model="POS"
            )
        #except:
            # If this fails there is nothing to return.
        #    rospy.logerr(f"[POS] Cannot decode results")
            
        #self.timer.time("Graph Nodes Edges")

    def draw_graph(self):
        # Draw the Poses on the provided image 'image' and return
        # the composite 'graph'
        self.timer.lap("Draw Poses")
        try:
            self.canvas = draw_poses(
                img = self.canvas, 
                poses = self.graph, 
                point_score_threshold = 0.1) #elf.parameters.threshold

        except:
            rospy.logerr(f"[POS] Cannot draw Poses")
        self.timer.time("Draw Poses", name="POS")

    def action_loop(self):

        # First, we need to decide if we are using an existing 
        # segmentation mask, or creating a new segmentation. If there
        # are new masks, we will update the self.segmentation_graph
        # and then remove our flag since we can use this until the new
        # results are delivered.
        if self.new_mask:
            self.update_segmentation_graph()
            self.new_mask = False
        
        # Next we will decide if we are recalculating the existing
        # graph nodes at self.graph.
        if self.new_graph:
            self.calculate_pose_graph()
            self.new_graph = False
        
        # If we are combining the segmentation and the images, then we
        # first need to mask the new images, but we only need to do
        # this if we need to combine the images and the inferences.
        temp = True
        if temp: #self.parameters.image_output:
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
            # If we are not using the camera image output then we
            # will just replace the self.image with the segmentation
            # graph image.
            
                self.canvas = self.segmentation_graph
            else:
                self.canvas = np.zeros((
                    self.height, 
                    self.width, 
                    3
                    ), dtype = "uint8"
                    )
            
        # Next we will draw our the current pose graph nodes and edges
        # onto our canvas
        
        #if self.parameters.posedetectionmodel == True:
        try:
            self.draw_graph()
        except:
            rospy.logerr("[POS] Error drawing pose detection graph")

        # Finally we will convert our image canvase back to the 
        # ROS message format.
        try:
            image_ros = self.bridge.cv2_to_imgmsg(self.canvas, 'bgr8')
            return image_ros
                
        except (CvBridgeError, TypeError, UnboundLocalError) as e:
            # If this conversion failes we will simply provide the last
            # good ROS image message since sometimes packages are lost
            rospy.logerr("[IMG]Error converting to ROS msg")
            print (e)
            return self.image

    def loop(self):

        rate = rospy.Rate(self.parameters.rate)
        while not rospy.is_shutdown():  
            # We have created a parameter for dynamically updateding an
            # active node. These will update if the global value changes
            #if self.parameters.updates == True:
            #    self.parameters.update()
            self.timer = ProcessTimer(abv = self.name.abv)
            ros_image = self.action_loop()
            self.pub.publish(ros_image)
            rate.sleep()

if __name__ == '__main__':
    AugmentedVRModule()
