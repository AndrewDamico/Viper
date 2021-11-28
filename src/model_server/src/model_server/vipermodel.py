import roslib
import os

class ViperModel(object):
    # A ViperModel contains the location of the model architecture
    # and the model weights which are stored in the node package.
    
    def __init__(self, package_name, model_xml, weights_bin):
        self.pkg = package_name
        self.model = model_xml
        self.weights = weights_bin
        self.setup_model()
        
    def setup_model(self):
        self.dir = roslib.packages.get_pkg_dir(self.pkg)
        self.location = os.path.join(
            self.dir, 
            self.model
            )
        self.weights = os.path.join(
            self.dir, 
            self.weights
            )
