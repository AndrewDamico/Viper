#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class SubscriberNode(object):
    
    def __init__(self):
        self.setup_ros()
        self.setup_subscriber()
        self.main_loop()
    
    def setup_ros(self):
        rospy.init_node('sample_subscriber')
        self.message = String()
        
    def setup_subscriber(self):
        rospy.Subscriber(
            name = 'topic', 
            data_class = String, 
            callback = self.callback,
            queue_size = 1)
        
    def callback(self, msg):
        self.message = msg
        self.action_loop()
        
    def action_loop(self):
        text = self.message.data
        rospy.loginfo(f'I heard {text}')
        
    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    SubscriberNode()
