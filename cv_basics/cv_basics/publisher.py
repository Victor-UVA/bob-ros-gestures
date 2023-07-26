import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.declare_parameter("source", 0)
        source = self.get_parameter("source").value

        self.publisher_ = self.create_publisher(Image, "camera_" + str(source), 10)

        self.cap = cv2.VideoCapture(source)
        self.br = CvBridge()
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        timer_period = 1/fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info("Publishing video frame")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()