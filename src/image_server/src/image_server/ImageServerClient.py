import rospy
from std_msgs.msg import Bool


class ImageServerClient(object):
    def __init__(self, name: str):
        self._name = name
        self.waiting = Bool()
        self.remote_status = Bool()
        self.setup_publisher()
        self.setup_subscriber()
        
    def setup_publisher(self):
        self.pub = rospy.Publisher(
            f'image_request/{self._name}',
            Bool,
            queue_size=1
            )
        rospy.loginfo(
            f'[{self._name}] Image server request services online.'
            )
            
    def update(self, state: bool):
        self.waiting.data = state
        self.pub.publish(self.waiting)
   
    def status(self, entity = "server"):
        if entity == "server":
            return self.remote_status.data
        else:
            return self.waiting.data
    
    def refresh(self, state: bool):
        self.waiting.data = state
        
    def setup_subscriber(self):
        rospy.Subscriber(
            'image_server/status',
            Bool,
            self.callback,
            queue_size=1
            )
        rospy.loginfo(
            f'[{self._name}] Subscribed to Image Server.'
            )

    def callback(self, msg):
        self.remote_status = msg

