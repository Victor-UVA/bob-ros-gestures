import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

BaseOptions = mp.tasks.BaseOptions
FaceDetector = mp.tasks.vision.FaceDetector
FaceDetectorOptions = mp.tasks.vision.FaceDetectorOptions
FaceDetectorResult = mp.tasks.vision.FaceDetectorResult
VisionRunningMode = mp.tasks.vision.RunningMode

#this is a ROS node based on code from ml_face_detector.py
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(
            Image,
            "video_frames",
            self.listener_callback,
            10
        )

        self.options = FaceDetectorOptions(
            base_options = BaseOptions(model_asset_path="/home/evan/Desktop/Programming/ROS/ros2tutorials/src/cv_basics/cv_basics/blaze_face_short_range.tflite"),
            running_mode = VisionRunningMode.IMAGE
        )
        
        self.br = CvBridge()
    

    def listener_callback(self, data):
        self.get_logger().info("Receiving video frame")
        currentFrame = self.br.imgmsg_to_cv2(data)

        with FaceDetector.create_from_options(self.options) as detector:
            image = mp.Image(image_format = mp.ImageFormat.SRGB, data=currentFrame)
            result = detector.detect(image)

            finalImage = currentFrame
            for detection in result.detections:
                box = detection.bounding_box
                x = box.origin_x
                y = box.origin_y
                w = box.width
                h = box.height
                cv2.rectangle(finalImage, (x,y), (x+w,y+h), (0, 255, 0), 4)

        cv2.imshow("camera", currentFrame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    imageSubscriber = ImageSubscriber()
    rclpy.spin(imageSubscriber)
    imageSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()