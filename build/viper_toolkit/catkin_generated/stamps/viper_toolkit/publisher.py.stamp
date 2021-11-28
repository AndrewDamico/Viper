#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class PublisherNode(object):
    def __init__(self):
        self.setup_ros()
        self.setup_publisher()
        self.main_loop()
        
    def setup_ros(self):
        # Register our node
        rospy.init_node('viper_toolkit/publisher')
 
    def setup_publisher():
        # Create our publisher
        pub = rospy.Publisher(
            name = 'topic',
            data_class = String,
            queue_size = 1)
    
    def action_loop(self):
        # This is generally the action that the node is performing.
        time = rospy.get_time

        return  time
    
    def main_loop(self):
        # This is the main loop which continually runs while the node
        # is online.
        
        # Set the minimum rate at which the node will run in hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            time = self.action_loop()
            message = f'The time is now {time}'
            pub.publish(message)
            rate.sleep()

if __name__ == '__main__':
    publisher()
