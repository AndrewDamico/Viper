#!/usr/bin/env python3
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

############################# STD MODULES ##############################
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from viper_toolkit import ProcessTimer, NameManager
from viper_toolkit import Parameter, ParameterManager
from viper_toolkit import Logger

######################### IMAGE SERVER MODULES #########################
from model_server.srv import ImageRequest, ImageRequestResponse
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

######################### MODEL SERVER MODULES #########################
from model_server import ViperModel
from model_server.srv import ModelRequest, ModelRequestResponse
from model_server.msg import InferenceResults
from std_msgs.msg import Int32, Float32
from array import array

##### Pose Detection
from pose_detection_module import PoseDetectionModel

##### Scene Segmentation
from scene_segmentation_module import SceneSegmentationModel

########################### OpenVino MODULES ###########################
from openvino.inference_engine import IENetwork, IECore

class ModelServer(object):
    
    def __init__(self):
        self.setup_ros()
        self.image = Image()
        self.setup_inference_engine()
        self.setup_models()
        self.loop()
        
    def setup_ros(self):
        rospy.init_node('model_server', log_level=rospy.DEBUG)
        self.setup_parameters()
        self.logger.i('Model Server Initialized')
    
    def setup_parameters(self):
        # Instantiates the Name Manager and Logger to standardize the 
        # nodes name and abreviation throughout the addon modules
        self.name = NameManager()
        self.logger = Logger(self.name)
        
        # Creates an instance of the organizer I will be using for 
        # my parameters. This allows for dynamic updatign accross nodes
        self.parameters = ParameterManager()
        
        # Sets this module to allow for dynamic updating
        self.parameters.add(
            Parameter(
                name="updates", 
                target=f"{self.name.name}/dynamic",
                default=True, 
                dynamic=True))
                
        # Searches for the code of the Pode Detection Module. If the 
        # the module cannot be found, the default of PDM will be used
        # and will be updated if the modules code changes when it comes
        # online. This code is used to tell the inference engine which
        # neural network to use for inference.
        self.parameters.add(
            Parameter(
                name="PoseDetection_abv", 
                target=rospy.search_param(
                    'pose_detection_module/abv'), 
                default="PDM", 
                dynamic=True))
        
        # Searches for the code of the Scene Segmentation module. Will
        # use default value if not found, and will update dynamically 
        # if the code later becomes available.
        self.parameters.add(
            Parameter(
                name = "SceneSegmentation_abv", 
                target = rospy.search_param(
                    'scene_segmentation_module/abv'), 
                default = "SEG", 
                dynamic = True))

    def setup_inference_engine(self):
        
        # Sets up the OpenVino Inference Engine core to handle
        # multiple models
        self.ie = IECore()
        self.logger.i("IECore Online")

    def setup_models(self):
        
        # Create a dictionary of models. The specific model will be
        # be identified at the time of inference.
        self.models = {}
        
        # Initializes the models. All models will be loaded to the 
        # Neural Compute Stick Vision Processing Unit.
        self.setup_pose_detector(device="MYRIAD")
        self.setup_scene_segmentor(device="MYRIAD")

    def setup_pose_detector(self, device):
        
        # Initializes a viper model pointing to the Artificial Neural
        # Network Architecture and the weights.
        pose_detector = ViperModel(
            package_name = 'pose_detection_module',
            model_xml = 'model/human-pose-estimation-0001.xml',
            weights_bin = 'model/human-pose-estimation-0001.bin'
            )
        
        # These model paramters are then loaded into the Intel
        # Nerual Compute Stick inference engine. 
        self.pose_engine = PoseDetectionModel(
            ie = self.ie,
            viper_model = pose_detector,
            device = device,
            model_tag = "PD.",
            model_name = "PoseDetectionModel"
            )
        
        # The path to the inference function of the model is 
        # saved in the model dictionary. This will be called when
        # a node makes a service API request via its access point.
        
        #self.models[f'{self.parameters.PoseDetection_abv}'] \
        #    = self.pose_engine.run_pose_estimation
            
        self.models["POS"] = self.pose_engine.run_pose_estimation
        self.logger.i('Pose Detection Model Loaded')

    def setup_scene_segmentor(self, device):
        
        # Here we set up the Scene Segmentation model.
        scene_segmentor = ViperModel(
            package_name = "scene_segmentation_module",
            model_xml = 'model/road-segmentation-adas-0001.xml',
            weights_bin = 'model/road-segmentation-adas-0001.bin'
            )
            
        # We load the model to the same device, which had been 
        # already been initialized at the server startup. This allows
        # us to add and remove models as needed, without shutting
        # the VPU down, while also sharing the device.
        self.scene_segmentor = SceneSegmentationModel(
            ie = self.ie,
            viper_model = scene_segmentor,
            device = device,
            model_tag = "SD.",
            model_name = "SceneSegmentationModel"
            )
        
        # We load the Scene Segmentation model to the model dictionary.
        
        #self.models[f'{self.parameters.SceneSegmentation_abv}'] \
        #    = self.scene_segmentor.run_scene_segmentation
        
        self.models["SEG"] = self.scene_segmentor.run_scene_segmentation
        self.logger.i("Scene Segmentor Loaded")

    def handle_model_rqst(self, request):
        
        # This function is the API callback. Upon receiving a proper
        # request via the service endpoint, the requested model
        # is saved as 'model'
        
        self.process_timer.lap("Modeling Service")
        self.logger.i("Modeling Request Received")
        model = request.model.data
        print(model)
        # The image which we will be performing inference on is 
        # is converted from a ROS message to a  numpy image array.
        try:
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(
                self.image,
                desired_encoding="rgb8"
                )
                
            #self.logger.i(f'Image reshaped to: {np.shape(image)}')
            
        except CvBridgeError as cv_error:
            print("ERRRRRRERERRROROR")
            self.logger.e(f'"CvBridge Error: {cv_error}')
            
            print(cv_error)
            
            #If this process fails we will need to return to try 
            # on the next loop. This sometimes happens due to 
            # packet loss.
            
            return
            
        # We use the resulting image to perform inference.
        self.process_timer.lap(model)
        
        try:
            results = self.models[model](frame=image)
            print(results)
        except:
            self.logger.e(f"Error modeling {model}")
            return
            
        self.process_timer.time(model)
        self.process_timer.lap(f'Flatten {model}')
        
        # We need to serialize all of the inferences accross models and
        # accross results in order to format them in a standard
        # and consistent message packet.
        try:
            packet = self.flatten_inference(results)
        except:
            self.logger.e(f"Error flattening {model}")
            
            return
            
        self.process_timer.time(f'Flatten {model}')
        self.process_timer.time("Modeling Service")
        
        # Upon success we will return the message packet to the API for
        # delivery to the client.
        
        return packet

    def handle_fetch_rqst(self, request):
        
        # This is the Service API callback for the Image Server Service.
        # Upon receiving a request from the model server to deliver
        # an image, this process saves that image message (self.image).
        #self.process_timer.lap("Image Service")
        #self.logger.i("Image request received")
        
        try:
            self.image = request.image
            receipt = True
        except:
            receipt = False
            
        #self.process_timer.time("Image Service")
        # We then respond with a message indicating the outcome of the
        # request.
        
        return ImageRequestResponse(Bool(receipt))

    def flatten_inference(self, results_array):
        
        # This function converts our results array into a 
        # [std_msgs.msg/int32] array containing the ndarry shape and 
        # a [std_msgs.msg/float32] array containing the flattened array.
        
        # Load the InferenceResults Template
        packet = InferenceResults()
        print(results_array)
        try:
            structure = results_array.shape
            rospy.logdebug("[MOD] mask structure: %s",structure)
            structure = list(structure) 
            structure = np.array(structure).astype(np.int32)
            structure = list(structure)
            rospy.logdebug("[MOD] Converted params: %s", structure)
            packet.structure = structure # [Int32]
            rospy.logdebug("[MOD] Structure sent: %s",packet.structure)
            
        except:
            rospy.logerr("[MOD] Cannot save mask structure.")
            
        try:
            inferences = np.array(list(results_array.flatten()))
            packet.inferences = inferences # [float32]
            
        except:
            rospy.logerr("[MOD] Cannot save Inference shape.")
            rospy.logerr("[MOD] Inference shape is: %s", inferences.shape)
        print(packet)
        return packet

    def loop(self):
        
        while not rospy.is_shutdown(): 
            
            # This function will keep our parameters dynamically updated
            # in case that they change as other nodes come online
            # or offile. Parameters marked as "updates = True" will
            # update accordingly. 
            
            if self.parameters.updates == True:
                    self.parameters.update()
            
            # This function Initializes my custom process timer module, 
            # which computes the time between any two proceses by name.
            # It also connects with my custom "Logger" module, the 
            # "Parameter" module, and my "NameManager" module to help me
            # track information accross Nodes.
            
            self.process_timer = ProcessTimer(logger = self.logger)
            
            # This is the API access point for the Image Server. The
            # client submits an Image Request message, which contains
            # an ROS Image message. In return they will receive a Bool
            # indicating the outcome of the request.
            
            self.image_fetching_service = rospy.Service(
                'model_server/image_fetching_service', 
                ImageRequest, 
                self.handle_fetch_rqst
                )
            #self.logger.i("Image Fetching Service Online")
            
            # This is the API access point for the Modeling Service. The
            # client submits a three letter model code, and the service
            # will use the most recent image to perform inference. It
            # then returns two arrays; one array 'Structure' contains 
            # the shape of the original inference output, and a second
            # array "Inferences" contains the flattened 1-Dimnesional 
            # array.
            
            self.modeling_service = rospy.Service(
                'model_server/modeling_service', 
                ModelRequest, 
                self.handle_model_rqst
                )
                
            #self.logger.i("Modeling Service Online")

            #self.logger.i("Model Server Online")
            
            # Rospy.spin() keeps our kernal active during times of 
            # inactivity.
            
            rospy.spin()

if __name__ == "__main__":
    ModelServer()
