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
############################# STD MODULES ##############################

from viper_toolkit import ProcessTimer, NameManager
from viper_toolkit import Parameter, ParameterManager
from viper_toolkit import Logger

######################### MODEL SERVER MODULES #########################
from model_server import ViperModel

########################### OpenVino MODULES ###########################
from openvino.inference_engine import IENetwork, IECore


class NeuralNetworkLoader(object):
    """
    
    A NeuralNetworkLoader is a class object which loads a pretrained
    model (architecture and weights) onto a initialized OpenVino
    inference engine.
    
    Keyword Arguments:
    
    ie -- an Inference Engine instance set up in the parent node which 
    we will load this model to.
    
    ViperModel -- a instance of class ViperModel which contains the 
    models weights and the structure of the neural network.
    
    device -- the inference device to be used to predict on (i.e., 
    "MYRIAD", CPU, GPU, etc.)
    
    model_tag -- a three letter abbreviation used by the VIPER Logger
    module which identifies log messages as originating from within this
    modules code.
    
    model_name -- a logger attribute which identifies this model.
    
    """
    
    def __init__(self, 
                ie: IECore, 
                viper_model: ViperModel,
                device: str,
                model_tag: str = "M..",
                model_name: str = "Model",
                *args,
                **kwargs):
        
        # Creates our helper tools from our Viper Toolkkit such as 
        # the parameter manager, our log manager, and our timer.
        self.setup_parameters(
            model_name = model_name, 
            model_tag = model_tag)
        
        # Prepare this model for loading
        self.setup_inference_engine(
            ie = ie, 
            viper_model = viper_model, 
            device = device)

        # Load the read network onto the initialized device.
        self.load_inference_engine(device=device, ie = ie)
        
        # Retrieve the architecture of the model to load, including 
        # inputs and outputs and stores these on the parameter server
        self.get_network_info()
        
        # Retrieves the image shapes for the input and output from the
        # now loaded model and stores these on the parameter server
        self.get_model_info()


    def setup_parameters(self, model_name: str, model_tag: str):
    
        # Instantiate our logger tool naming these processes and
        # setting the tag. The ("XX.") convention indicates this is a 
        # model and log messages are coming from within the 
        # model processing script and not the main node.
        self.logger = Logger(
            name = model_name, 
            tag = model_tag)
        
        # Instantiate our timer tool which will output the times of
        # the processes within the model, and indicate that the 
        # process originated from within the model, and not the module.
        self.timer = ProcessTimer(logger=self.logger)
        
        # Creates a parameter manager
        self.NeuralNetworkParams = ParameterManager(logger=self.logger)
        
    def setup_inference_engine(self, ie: IECore, viper_model: ViperModel, device: str):

        # Link the internal inference engine with the initialized engine
        # and read the network architecture.
        self._ie = ie
        
        # Load the Viper Model class object, which contains the address
        # for the neural network architecture and well as the weights
        # of the trained model.
        self._net = ie.read_network(
            model=viper_model.location,
            weights=viper_model.weights
            )

    def load_inference_engine(self, device, ie):
        
        # Load the network architecture and weights into the initialized
        # inference engine. We must indicate the device name which 
        # is passed through the main node.
        self._exec_net = ie.load_network(
            network = self._net,
            device_name = device
            )
            
    def get_network_info(self):
        
        # Set the input and output blobs
        self._input_blob = next(iter(self._exec_net.input_info))
        self._output_blob = next(iter(self._exec_net.outputs))

        # Get the input shape
        #self._input_shape = self._net.inputs[self._input_blob].shape
        #self.logger.i(f'Input shape: {self._input_shape}')
        
        # Save these parameters to the parameter server
        #self.NeuralNetworkParams.add(
        #    Parameter(
        #        name = "Input_shape",
        #        value = self._input_shape,
        #        dynamic = False))
            
        # Get the output shape
        self._output_shape = self._net.outputs[self._output_blob].shape
        self.logger.i(f'Output shape: {self._output_shape}')
        
        # Save these parameters to the parameter server
        self.NeuralNetworkParams.add(
            Parameter(
                name="Output_shape",
                value=self._output_shape,
                dynamic=False))
                
    def get_model_info(self):
        
        # Accesses the shape of the input layer and the output layer
        self._input_key = list(self._exec_net.input_info)[0]
        self._output_keys = list(self._exec_net.outputs.keys())
        self._tensors = self._exec_net.input_info[self._input_key].tensor_desc
        
        # Saves the shapes to variables representing
        self.n, self.c, self.h, self.w = self._tensors.dims
        self.logger.i(f'Tensor shape (NCHW): ({self.n}, {self.c}, {self.h}, {self.w})')
        
        self.NeuralNetworkParams.add(
            Parameter(
                name="Input_height",
                value=self.h,
                dynamic=False))

        self.NeuralNetworkParams.add(
            Parameter(
                name="Input_width",
                value=self.w,
                dynamic=False))

if __name__ == "__main__":
    NeuralNetworkLoader()
