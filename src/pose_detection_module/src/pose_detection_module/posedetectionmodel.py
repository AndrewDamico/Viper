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
from viper_toolkit import Parameters, Parameter
from viper_toolkit import Logger

############################# STD MODULES ##############################
import numpy as np
import cv2

######################### IMAGE SERVER MODULES #########################
from image_server import ImageServerClient

######################### MODEL SERVER MODULES #########################
from model_server import NeuralNetworkLoader
from model_server import ViperModel

######################### POSE_DETECTION_MODULE ########################
from pose_detection_module import OpenPoseDecoder
from pose_detection_module import heatmap_nms, pool2d

########################### OPENVINO MODULES ###########################
from openvino.inference_engine import IENetwork, IECore


class PoseDetectionModel(NeuralNetworkLoader):
    """
    A PoseDetectionModel is a class object of the type NeuralNetworkLoader
    which connects this model to the parent inference engine, communicates
    the neural network shape, and sets up logging.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.load_decoder()
        
    def load_decoder (self):
        # Load the model decoder which will decode the results of our 
        # ANN into nodes and edges representing a human being.
        self.decoder = OpenPoseDecoder()
        #self.logger.i("Decoder loaded")
        
    def process_results(self, frame, results):
        # the process_results function takes an image, the results inference
        # the output_keys of the model, the inference engine, 
        # and a decoder which contains the shape of the entity (i.e., human)
        # and gets poses from results
        #self.timer.lap("Processing Results")
        rospy.loginfo("process results")
        pafs = results[self._output_keys[0]]
        #print ("PAFS")
        #print(pafs)
        heatmaps = results[self._output_keys[1]]
        #print("HEATMAPS")
        #print(heatmaps)
        
        # The heatmaps are generated from the image by using Max pooling,
        # which takes the maximum value of the pixels contained within 
        # the shape of the kernel.
        # https://github.com/openvinotoolkit/open_model_zoo/blob/master/demos/common/python/models/open_pose.py
        rospy.loginfo("pool heatmaps")
        pooled_heatmaps = np.array(
            [[pool2d(h, 
                    kernel_size=3, 
                    stride=1, 
                    padding=1, 
                    pool_mode="max"
                    ) for h in heatmaps[0]]]
                    )
        print (pooled_heatmaps)
        nms_heatmaps = heatmap_nms(heatmaps, pooled_heatmaps)
        print("NMS HEATMAPS ===========================")
        print(nms_heatmaps)
        #self.timer.time("Processing Results")
        #self.timer.lap("Decoding Poses")
        rospy.loginfo("decode")
        # The generated heatmaps are then sent to the decoder
        
        poses, scores = self.decoder(
            heatmaps, 
            nms_heatmaps, 
            pafs
        )
        print("post decode")
        print(poses)
        print(scores)
        
        output_shape = self._exec_net.outputs[self._output_keys[0]].shape
        print("-------------outputshape")
        print (output_shape)
        
        output_scale = frame.shape[1] / output_shape[3], frame.shape[0] / output_shape[2]
        print("---------output scale")
        print(output_scale)
        #rospy.loginfo(f"Frame width, height: ({frame.shape[0]}, {frame.shape[1]})")
        #self.logger.i(f"Output width, height: ({output_shape[2]}, {output_shape[3]})")
        #self.logger.i(f"Output_scale: {output_scale}")
        
        # multiply coordinates by scaling factor
        poses[:, :, :2] *= output_scale
        #self.timer("Decoding Poses")
        return poses, scores


    def run_pose_estimation(self, frame):
        # Model Preperation
        # Initializing my timer module which will calcualtes the processing time
        # between any two 'tags'
        #self.timer =  ProcessTimer(logger = self.logger)
        #self.timer.lap("Image Processing")
        
        # Scaling the model
        scale = 1280 / max(frame.shape)
        #rospy.loginfo("scale: %s", scale)
        rospy.loginfo("init")
        if scale < 1:
            frame = cv2.resize(
                frame, 
                None, 
                fx=scale, 
                fy=scale, 
                interpolation=cv2.INTER_AREA
                )
        
        # resize image and change dims to fit neural network input
        # (see https://github.com/openvinotoolkit/open_model_zoo/
        #  tree/master/models/intel/human-pose-estimation-0001)
        input_img = cv2.resize(
            frame, 
            (self.w, self.h), 
            interpolation=cv2.INTER_AREA
            )
        rospy.loginfo("resized")
        #self.logger.i(f"Input Shape: {input_img.shape}")
        
        # create batch of images (size = 1)
        input_img = input_img.transpose(2, 0, 1)[np.newaxis, ...]
        rospy.loginfo("transposed")
        #self.logger.i(f"Transposed Shape: {input_img.shape}")
        
        # Printing Image Processing Time
        #self.timer.time("Image Processing")
        
        # Performing Model Inference
        # Performing inference and receiving the results.
        #self.timer.lap("Inference")
        results = self._exec_net.infer(
            inputs={self._input_key: input_img})
        rospy.loginfo("infer")
        #self.logger.i(f"Results Shape: {results.shape}")
        #self.timer.time("Inference")
        print(results)
        # get poses from network results
        poses, scores = self.process_results(frame=frame, results=results)
        #self.logger.i(f"Returned Shape: {poses.shape}")
        print ("------------------------------------")
        print (scores)
        print (poses)
        # Outputing time of tag 'total_runtime' which was created upon
        # timer initialization.)
        #self.timer.time(total_runtime)
        
        return poses #, scores
        

            
            
