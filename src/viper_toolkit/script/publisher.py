#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class PublisherNode(object):
    def __init__(self):
        self.setup_ros()
        self.setup_publisher()
        self.main_loop()
        
    def setup_ros(self):
        rospy.init_node('sample_publisher')
 
    def setup_publisher(self):
        self.pub = rospy.Publisher(
            name = 'topic',
            data_class = String,
            queue_size = 1)
    
    def action_loop(self):
        time = rospy.get_time()
        return  time
    
    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            time = self.action_loop()
            message = String(f'The time is now {time}')
            self.pub.publish(message)
            rospy.loginfo(f'I published: {message}')
            rate.sleep()

if __name__ == '__main__':
    PublisherNode()
