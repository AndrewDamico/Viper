#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class SubscriberNode(object):
    
    def __init__(self):
        setup_ros()
        setup_subscriber()
        main_loop()
    
    def setup_ros(self):
        # Register our node
        rospy.init_node('viper_toolkit/subscriber')
        
        # Create our message container. This allows us to retrieve the
        # previous message upon failure so that we do not disrupt 
        # the system loop in the event of a dropped message.
        self.message = String()
        
    def setup_subscriber(self):
        # Setup a subscription to the topic 'topic.' We will then receive
        # any messages sent to this topic.
        rospy.Subscriber(
            name = 'topic', 
            data_class = String, 
            callback = self.callback)
        
    def callback(self, msg):
        # This tells the node what to do when it receives a new message.
        # In this example we are asking it to save the message to the 
        # container we instantiated earlier in the setup_ros() function.
        self.message = msg
        # Here, rather than perform our action in the main loop, which
        # is activated by cycle, we will have our action originate from
        # the callback, which is activated whenever a message is received
        # and hence the callback is called.
        self.action_loop()
        
    def action_loop(self):
        # We will have this node take a message, and then make it all 
        # capital letters. We first get the property we saved to 
        # self.message... in messages of type 'String' the message text 
        # is saved in the 'data' property.
        text = self.message.data
        # We then output the results to the log and the terminal
        rospy.loginfo(f'I heard {result}')
        
    def main_loop(self):

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    subscriber()
