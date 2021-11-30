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
from image_server import ImageServerClient

######################### MODEL SERVER MODULES #########################
from model_server.srv import ModelRequest, ModelRequestResponse
from model_server.msg import InferenceResults
from std_msgs.msg import Float32, String

##### Pose Detection

##### Scene Segmentation
from scene_segmentation_module import segmentation_map_to_image

########################### OpenVino MODULES ###########################
#from openvino.inference_engine import IENetwork, IECore

######################### AUGMENTED VR MODULES #########################
#from augmented_vr_module import draw_poses

class SceneSegmentationModule(object):
    def __init__(self):
        self.name = NameManager(abv="SEG")
        self.inference = InferenceResults()
        self.setup_ros()
        #self.setup_model()
        self.loop()
    
    def setup_parameters(self):
        self.parameters = ParameterManager()
        self.parameters.add(Parameter(
            name="rate", 
            target=f"{self.name.name}/rate", 
            default=50, 
            dynamic=True
            ))
        self.parameters.add(Parameter(
            name="updates", 
            target=f"{self.name.name}/dynamic", 
            default=True, 
            dynamic=True
            ))

    def setup_ros(self):
        self.setup_parameters()
        rospy.init_node('scene_segmentation', log_level=rospy.DEBUG)
        self.setup_model_server()
        self.image_server = ImageServerClient(self.name.abv)

    def setup_model_server(self):
        
        self.model_request = rospy.ServiceProxy(
            'model_server/modeling_service',
            ModelRequest,
            )

        self.pub = rospy.Publisher(
            'model_output/scene_segmentation/segmentation_mask',
            InferenceResults,
            queue_size=1
            )

    def image_server_backup(self):
        self.waiting = Bool
        self._image_server_Status = Bool
        #Sends out if it is waiting for images
        self.wait_status.pub = rospy.Publisher(
            'image_request/SEG',
            Bool,
            queue_size=1
            )
        rospy.Subscriber('image_server/status',
            Bool, 
            self.image_server.callback, 
            queue_size=1
            )
        
        def callback(self, msg):
            self._image_server_Status = msg

    def action_loop(self):
        self.timer.lap("Wait for Service")
        rospy.wait_for_service(
            'model_server/modeling_service'
            )
        self.timer.time("Wait for Service")
        try:
            self.timer.lap("Modeling Server")
            rospy.logdebug(f"[{self.name.abv}] Service Requested.")
            req = String()
            req.data = "SEG" #self.name.abv
            response = self.model_request(req)
            self.timer.time("Modeling Server", name = "MOD")
            self.inference = response.results
            rospy.logdebug(
                f"[{self.name.abv}] Mask shape received: {self.inference.structure}"
                )
            return self.inference

        except rospy.ServiceException as e:
            rospy.logerr(f"[{self.name.abv}] Service call failed: {e}")
            

    def loop(self):
        rate = rospy.Rate(self.parameters.rate)
        while not rospy.is_shutdown():
            if self.parameters.updates == True:
                self.parameters.update()
                
            # Setup timer. Note that this resets on every iteration. 
            # Future development we may want to move this so that we can 
            # compute averages.
            
            self.timer = ProcessTimer(abv = self.name.abv)
            
            # Check to see if the server has released a new image and 
            # if so, start new modeling request
            
            status = self.image_server.status("server")
            rospy.logdebug(
                f"[{self.name.abv}] Remote Image Server Status is: {status}"
                )
            if status == True: 
                # Takes down image request flag while performing 
                # inference since inference time is not negligable
                
                self.image_server.update(False)
                
                my_status = self.image_server.status("me")
                rospy.logdebug(
                    f"[{self.name.abv}] Waiting Status: {my_status}"
                    )
                # Create the image mask
                try:
                    mask = self.action_loop()
                    self.pub.publish(mask)
                    rospy.logdebug(f"[{self.name.abv}] Mask Published")
                except:
                    pass
                print (mask)
            # Update the image server to let it know we need a new image
            
            self.image_server.update(True)

            rospy.logdebug(
                "[SEG] Waiting Status: %s", 
                self.image_server.status("me")
                )
            self.timer.time("total_runtime")
            rate.sleep()
            
if __name__ == '__main__':
    SceneSegmentationModule()
