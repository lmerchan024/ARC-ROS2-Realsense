import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class IntelSubscriber(Node): #create intel subscriber class
    def __init__(self):
        super().__init__("intel_subscriber")
        self.subscription_rgb = self.create_subscription(Image, "rgb_frames", self.rgb_frame_callback, 10) #sub type is Image, looking for 'rgb_frame', callback? ,q size 10
        self.br_rgb = CvBridge() #bridge between cv and ros?


    def rgb_frame_callback(self, data): #define rgb frame callback, takes in data
        self.get_logger().warning("Receiving RGB frame") #
        current_frame = self.br_rgb.imgmsg_to_cv2(data) #convert msg to cv2 data
        cv2.imshow("RGB", current_frame) #then display that data
        cv2.waitKey(1)




def main(args = None): #main function
    rclpy.init(args = args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber) #what does this do? what is spin?
    intel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()