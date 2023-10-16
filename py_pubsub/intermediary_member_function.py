'''
This node is a TEMPLATE for a middleman node that
listens to a topic with text, edits the message, then edited message
alias is middleman
'''
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
#%% Node Creation
class MinimalSubPub(Node):

    def __init__(self):
        super().__init__('minimal_middleman') #node name
        
        #listen to the topic 'topic' and send it to the 'listener_callback' function
        self.subscription = self.create_subscription(String,'topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning?

        #create publisher
        self.publisher_=self.create_publisher(String, 'Relay',10) #create new topic called 'Relay'


    #Functions that will be called upon
    def listener_callback(self, msg):
        modified_data=msg.data+' I am a middleman node!'
        self.get_logger().info('Received: "%s", Publishing: "%s"' % (msg.data, modified_data))

        new_msg=String()
        new_msg.data=modified_data
        self.publisher_.publish(new_msg)

#%% Running the node
def main(args=None):
    rclpy.init(args=args)

    minimal_intermediary=MinimalSubPub() #call upon the node
    rclpy.spin(minimal_intermediary)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_intermediary.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
#%%