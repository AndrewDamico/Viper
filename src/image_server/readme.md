# Image Server
The Image Server module helps to throttle the images being produced by the camera to minimize bandwidth and processing while still providing each node with the images they need at the time that they need it.

Originally, each ROS module was a subscriber to their own images, which came directly from the camera. In order to process a model, the module would take the last image message and send it to the model server for inference. 


1. This meant that not only would the modules need to create a copy of the image, but they would also need to send that image to the model server, which cost both time and bandwidth. 
2. The model server might be receiving duplicates of the same image and have to perform processing twice. 
3. The model server might also be perform processing on two seperate images which were taken milliseconds apart.
4. During the time that the model server is performing inference for another model it is unable to receive requests and the other requesting modules need to wait. By the time these modules reach their turn the images they are requesting inference on are stale. 

To resolve this I created the Image Server module. The Image Server module works by subscribing to "flags" for each of the modules. Whenever a module outs up a flag that it has received a new image inference (and hence ready for a new image) the image server delivers a new image to the model server. As soon as the model server accepts the image, the Image Server:
1. Broadcasts to all models that a new image is available, and they can make a model request.
2. Resets all models "Image Request" flags, including the ones made while it was delivering this last image. 

This has the effect of ensuring that models do not attempt to make a modeling request when there are no new images available, and that all models who make a request at the same time are able to use the same image, cutting down on resources.  

