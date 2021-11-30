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

############################# STD MODULES ##############################
import numpy as np
import cv2

######################### MODEL SERVER MODULES #########################
from model_server import NeuralNetworkLoader
from model_server import ViperModel


class SceneSegmentationModel(NeuralNetworkLoader):
    """
    
    A SceneSegmentationModel is a class object which uses a Convolutional
    Neural Network (CNN) to classify  every pixel within an image as a 
    member of a certain class (segments the image). In this  pytorch 
    pretrained model we are predicting on 4 classes: 
        (a) Roadway, (b) Curb, (c) Background, and (d) marker. 
    
    The model specifications can be found at: 
    https://docs.openvino.ai/2018_R5/_docs_Transportation_segmentation_curbs_release1_caffe_desc_road_segmentation_adas_0001.html
    
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def export_parameters(export_state=True, **kwargs):
        # Exports parameters sent to this function for debugging 
        # purposes, and then turns off function after export_state=False
        # is received.
        self.export_state = False
        
        for arg in kwargs:
            self.parameters.add(
                Parameter(
                    name = arg,
                    value = kwargs[arg],
                    dynamic = False))

        self.export_state = export_state

    def run_scene_segmentation(self, frame):

        # The default ROS image is BGR color, however the model is
        # expecting RGB, so we will need to convert this first.
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # We will now attempt to resize the image to fit the model input
        #try:
        input_img = cv2.resize(rgb_image, (self.w, self.h))
        rospy.loginfo("resized")
            #self.logger.d(f"Resized Shape: {input_img.shape}")
        #except:
            #self.logger.e(f"Cannot resize image to shape ({self.w}, {self.h})") 
            #return
            
       # if self.export_state: self.export_parameters(resized_shape = input_img.shape)

        # We need to wrangle the image from into the NCHW format.
        #try:    
        transposed_img = np.expand_dims(
                a = input_img.transpose(2, 0, 1),
                axis = 0)
        rospy.loginfo("transposed")
            #self.logger.d(f"Transposed Shape: {transposed_img.shape}")
        #except:
        #    self.logger.e("Error converting to NCHW format")
        #    return
        
        #if self.export_state: self.export_parameters(transposed_shape=transposed_img.shape)
        

        # We will now perform inference on the input object using the
        # inference engine we loaded to the Visual Processing Unit
        results = self._exec_net.infer(
            inputs = {self._input_key: transposed_img}
            )
        rospy.loginfo("infered")
        # Extract the inference blob from the results array
        result_ir = results[self._output_blob]
        
        # We then compute the maximum value along axis 1 indicating
        # the max likelyhood class for which that pixel belongs, and
        # return this classification map of the original image.
        mask = np.argmax(
            a = result_ir, 
            axis=1)
        
        # We export the successful shape and then turn off exporting.
        #if self.export_state:self.export_parameters(mask_shape=mask.shape, export_state=False)
            
        #self.logger.i(f"Returning shape: {mask.shape}")

        return mask
