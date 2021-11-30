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
from viper_toolkit import ParameterManager, Parameter
from viper_toolkit import Logger
#from std_msgs.msg import String, Bool, Float32, Int8
#from sensor_msgs.msg import Image

############################# STD MODULES ##############################
import time
import numpy as np
#import cv2
#from cv_bridge import CvBridge, CvBridgeError

######################### IMAGE SERVER MODULES #########################
from image_server import ImageServerClient

######################### MODEL SERVER MODULES #########################
from model_server.srv import ModelRequest, ModelRequestResponse
from model_server.msg import InferenceResults
from std_msgs.msg import String



class PoseDetectionModule(object):

    def __init__(self):
        self.name = NameManager()
        self.logger = Logger(name = self.name)
        self.pose_estimations = InferenceResults()
        self.setup_ros()
        self.loop()
        
    def setup_parameters(self):
        self.parameters = ParameterManager()
        
        self.parameters.add(
            Parameter(
                name="rate", 
                target=f"{self.name.name}/rate", 
                default=50, 
                dynamic=True))
                
        self.parameters.add(
            Parameter(
                name="updates", 
                target=f"{self.name.name}/dynamic", 
                default=True, 
                dynamic=True))


    def setup_ros(self):
        
        rospy.init_node('pose_detection_model', log_level=rospy.DEBUG)
        
        self.setup_parameters()
        
        self.model_request = rospy.ServiceProxy(
            'model_server/modeling_service',
            ModelRequest
            )

        self.pub = rospy.Publisher(
            'model_output/pose_articulator/human_graph_nodes',
            InferenceResults,
            queue_size=1
            )

        self.setup_image_server()

    def setup_image_server(self):
        self.image_server = ImageServerClient(self.name.abv)

    def action_loop(self):
        self.timer.lap("Wait for Service")
        rospy.wait_for_service('model_server/modeling_service')
        self.timer.time("Wait for Service")
        
        try:
            self.timer.lap("Modeling Server")
            req = String()
            req.data = "POS" #self.name.abv
            print (req)
            response = self.model_request(req)
            print(response)
            self.timer.time("Modeling Server")
            self.inference = response.results
            self.logger.i(
                f"Mask shape received: {self.inference.structure}")
            print(self.inference)
            return self.inference

        except rospy.ServiceException as e:
            self.logger.e(f"Service call failed: {e}")
            #return self.inference

    def loop(self):
        
        rate = rospy.Rate(self.parameters.rate)
        
        while not rospy.is_shutdown():
            
            if self.parameters.updates == True:
                self.parameters.update()

            self.timer = ProcessTimer(logger = self.logger)
            
            # Check to see if the server has released a new image and 
            # if so, start new modeling request
            
            status = self.image_server.status("server")
            
            self.logger.i(f"Remote Image Server Status is: {status}")
            
            if status == True: 
                
                # Takes down image request flag while performing 
                # inference since inference time is not negligable
                self.image_server.update(False)
                
                my_status = self.image_server.status("me")
                self.logger.i(f"Waiting Status: {my_status}")
                # Retrieve the pose estimations
                try:
                    estimates = self.action_loop()
                    
                    # Publish the estimations
                    self.pub.publish(estimates)
                    rospy.logdebug(f"[{self.name.abv}] Mask Published")
                except:
                    pass
            # (re)Publishes tag indicating that we (still) want new
            # images from the image server, either because we were 
            # successful, or because we are still waiting.
            self.image_server.update(True)
            
            self.logger.i(
                f"Waiting Status: {self.image_server.status('me')}")
            self.timer.time("total_runtime")
            rate.sleep()

if __name__ == '__main__':
    PoseDetectionModule()
