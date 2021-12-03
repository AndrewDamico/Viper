# The ROS VIPER Library

The VIPER Vision Inferencing Processing Endogenous Robot operates on Ubuntu 20.04 and utilizes Python and C++ programming languages. At its core it runs on the Robotic OS platform (ROS: 'Noetic' Release), and implements the Intel Neural Compute Stick 2, showcasing the OpenVino model optimization toolkit. 

The VIPER network Network consisting of 6 Nodes and
a Python Utilities package. One of the many reasons I chose ROS was due to the scalability of the Nodes and the network structure. Currently, all nodes are operated on the Viper "Parmenides" Device, however multiple devices could support multiple nodes within the same private network connected via WiFi or bluetooth, creating a mobile computing cluster and enabling multidimensional modeling. (One of the use cases I will be creating demonstrates how two devices can use their independent inferences to create additional network inferences).

The Viper Workspace contains the following packages:

* augmented_vr_module Aggregates the model outputs into a single
    output which can contain the original image (Augmented Reality) or only contain model output (Virtual Reality)
* image_server Manages the delivery of the video input to throttle for just-in-time delivery. The image_server tracks when nodes require new images to minimize waste (images which are deliveredbut dropped.
* inland_ir_cam Adapts the Raspicam module for use with the Inland IR Camera on the 64 Bit Linux Ubuntu System, running on the Raspberry Pi 4+
* model_server Manages the neural networks on the Intel Neural Compute Stick 2 Visual Processing Unit (VPU) by running the main instance of the OpenVino Inference Engine, and managing service requests to perform inference.
* pose_detection_module Uses the Pose_detection_model to infer the presence of humans and predict their posture
* scene_segmentation_model Takes an input and classifies the image blobs as one of four seperate classes.
* viper_toolkit A python module which contains multiple utilities build to help manage subsystems accross the different nodes, including log management, name management, parameter management, and process timer management.



## Installation

Package should be installed using the ROS package manager.


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[[BSD](https://github.com/AndrewDamico/viper/blob/main/LICENSE)
