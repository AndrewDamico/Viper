#!/usr/bin/env python
__package__ = "model_server"
__version__ = '0.5'
__author__ = 'Andrew Damico'

from .vipermodel import ViperModel
from .neuralnetworkloader import NeuralNetworkLoader

__all__ = [
    'ViperModel',
    'NeuralNetworkLoader'
]
