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



class AugmentedVRModule(object):
    
    def __init__(self):
        rospy.init_node('Augmented_VR', log_level = rospy.DEBUG)
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
        
        self.parameters.add(
            Parameter(
                name = "width", 
                target = "/inland_ir_cam/width", 
                default = 800, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "height", 
                target = "/inland_ir_cam/height", 
                default = 600, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "updates", 
                target = f"{self.name.name}/dynamic", 
                default = False, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "image_output", 
                target = f"{self.name.name}/image_output", 
                default = True, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "alpha", 
                target = rospy.search_param('alpha'), 
                default = 0.5, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "threshold", 
                target = rospy.search_param('threshold'), 
                default = 0.1, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "segmentationmodel", 
                target = f"{self.name.name}/segmentation", 
                default = True, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "posedetectionmodel", 
                target = f"{self.name.name}/posedetection", 
                default = True, 
                dynamic = False))
        self.parameters.add(
            Parameter(
                name = "rate", 
                target = f"{self.name.name}/rate", 
                default = 5, 
                dynamic = False))
        
    def setup_ros(self):
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
        self.new_graph = False
        # Important! Sets the first self.output to a black image since
        # we will call on this before the first actual output is ready
        self.output = self.canvas
        self.graph = []
        rospy.loginfo("Pose detector subscription active")

    def setup_scene_segmentation(self):
        self.mask = InferenceResults()
        self.new_mask = False
        rospy.Subscriber(
            'model_output/scene_segmentation/segmentation_mask',
            InferenceResults,
            self.scene_segmentation_callback,
            queue_size=1
            )
        # Important! Sets the first self.segmentation_graph to a black 
        # image since we will call on this before the first actual output
        #  is ready
        self.segmentation_graph = self.canvas
        self.mask = InferenceResults()
        self.colormap = np.array([[68, 1, 84], [48, 103, 141], [53, 183, 120], [199, 216, 52]])
        rospy.loginfo("Scene Segmentation subscription active")

    def pose_estimation_callback(self, msg):
        self.pose_estimations = msg
        self.new_graph = True
        rospy.loginfo("[POS] New Coordinates Received")

    def scene_segmentation_callback(self, msg):
        self.mask = msg
        self.new_mask = True
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
            rospy.logdebug(f'[{name}] Converting {type(results.structure)} to structure.')
            rospy.logdebug(f'[{name}] Structure: {structure}')
        except:
            rospy.logerr(
                f"[{name}] Cannot convert {type(self.results.structure)} to structure"
                )
            return

        try:
            inferences = np.asarray(results.inferences)
            rospy.logdebug(f"[{name}] Transformed shape: {inferences.shape}")
        except:
            rospy.logerr(f'[{name}] Cannot convert to numpy array')
            return
            
        try:
            inference_array = inferences.reshape(structure)
            rospy.logdebug(f'Wrangled Shape: {inference_array.shape}')
            rospy.logdebug(f'Blob Type: {type(inference_array)}')
        except:
            rospy.logerr(f"[{name}] Cannot wrangle data blob")
            return
            
        self.timer.time("Decode Inference", name=name)
        return inference_array
            
    def update_segmentation_graph(self):
        '''
        This function tasks a message file (self.mask) and decodes it into
        the original array, outputing the "segmentation graph". Note that
        this output can be used on its own as an image.
        '''
        self.timer.lap("Segmentation Process")
        
        # First, we will attempt to reshape the inference into its original
        # array shape
        try:
            inference_array = self.decode_inference_results(
                    results = self.mask, 
                    model = "SEG")
        except:
            rospy.logerr(f"[SEG] Cannot decode results")
            return
        
        # Next we will map the inference array onto a color map
        try:
            self.timer.lap("Segmentation Model")
            mask = segmentation_map_to_image(inference_array, self.colormap)
            rospy.logdebug(f"[SEG] Resizing Mask")
        except:
            rospy.logerror(f'[SEG] Cannot resize Mask.')
            return
        
        # Finally we will resize the color map to the same size as our 
        # original image.
        try:
            self.segmentation_graph = cv2.resize(
                    mask, 
                    (self.image.width, self.image.height))
            if self.parameters.image_output:
                self.segmentation_graph = img_as_ubyte(self.segmentation_graph)
            self.timer.time("Segmentation Model", name="SEG")
            self.timer.time("Segmentation Process", name="SEG")
            return
        except:
            rospy.logerr(f"[SEG] Cannot resize mask at point A")
            self.timer.time("Segmentation Model", name="SEG")
            self.timer.time("Segmentation Process", name="SEG")
            return

    def combine_segmentation_image(self):
        '''
        This function combines the current segmentation_graph 
        and the current converted image, and saves it as the 
        current output (canvas)
        '''
        alpha = self.parameters.alpha
        self.timer.lap("Image Overlay")
        try:
            self.canvas = cv2.addWeighted(
                self.segmentation_graph, 
                alpha,
                self.canvas, 
                1 - alpha, 
                0)
        except:
            rospy.logerr(f"[SEG] Cannot combine mask and image.")
            
        #Regardless of success we will update our timer.
        self.timer.time("Image Overlay", name = "SEG")
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
            self.output = draw_poses(
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
            # creates new self.segmentation_graph
            self.update_segmentation_graph()
            self.new_mask = False
        
        # Next we will decide if we are recalculating the existing
        # graph nodes at self.graph.
        if self.new_graph:
            # creates new self.graph
            self.calculate_pose_graph()
            self.new_graph = False
        
        # If we are combining the segmentation and the images, then we
        # first need to mask the new images, but we only need to do
        # this if we need to combine the images and the inferences.
        if self.parameters.image_output:
            if self.new_image:
                try:
                    # Convert the camera image to ROS
                    self.canvas = self.bridge.imgmsg_to_cv2(
                        self.image, 
                        desired_encoding="bgr8"
                        )
                    # set the new image to false in case this node runs
                    # faster than the image server.
                    self.new_image = False
                except CvBridgeError as e:
                    # if this fails then give us the last good image.
                    rospy.logerr("[IMG] Error converting ROS msg to image.")
                    print(e)
                # We check to see if the segmentation model needs to be printed
                if self.parameters.segmentationmodel:
                    try:
                        # We can now combine them on the 'self.canvas' layer.
                        self.combine_segmentation_image()
                    except:
                        rospy.logerr("[SEG] Error proccessing scene")
        # If image output is False:
        else:
            if self.parameters.segmentationmodel:
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
        
        if self.parameters.posedetectionmodel:
            try:
                self.draw_graph()
            except:
                rospy.logerr("[POS] Error drawing pose detection graph")
            
            # Finally we will convert our image canvase back to the 
            # ROS message format.
            try: 
                image_ros = self.bridge.cv2_to_imgmsg(self.output, 'bgr8')
            except (CvBridgeError, TypeError, UnboundLocalError) as e:
                rospy.logerr("[POS] Cannot return output in ROS format.")
                rospy.logerr(e)
                return self.image
            
        else:
            # Finally we will convert our image canvase back to the 
            # ROS message format.
            try:
                image_ros = self.bridge.cv2_to_imgmsg(self.canvas, 'bgr8')
            except (CvBridgeError, TypeError, UnboundLocalError) as e:
                rospy.logerr("[POS] Cannot return output in ROS format.")
                rospy.logerr(e)
                return self.image
        
        return image_ros

    def loop(self):

        rate = rospy.Rate(self.parameters.rate)
        while not rospy.is_shutdown():
            # We have created a parameter for dynamically updating an
            # active node. These will update if the global value changes
            if self.parameters.updates == True:
                self.parameters.update()
            self.timer = ProcessTimer(abv = self.name.abv)
            ros_image = self.action_loop()
            self.pub.publish(ros_image)
            rate.sleep()

if __name__ == '__main__':
    AugmentedVRModule()
