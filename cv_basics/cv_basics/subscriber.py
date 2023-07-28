import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions
import time
from ament_index_python.packages import get_package_prefix

from cv_basics_messages.msg import FaceDetection
from cv_basics_messages.msg import FaceDetectionArray
from cv_basics_messages.msg import Landmark
from cv_basics_messages.msg import FaceLandmarkArray
from cv_basics_messages.msg import FaceLandmarkArrayArray
from cv_basics_messages.msg import Hand
from cv_basics_messages.msg import HandArray

#basics
BaseOptions = mp.tasks.BaseOptions
VisionRunningMode = mp.tasks.vision.RunningMode

#face detection
FaceDetector = mp.tasks.vision.FaceDetector
FaceDetectorOptions = mp.tasks.vision.FaceDetectorOptions
FaceDetectorResult = mp.tasks.vision.FaceDetectorResult

#face landmarking
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
FaceLandmarkerResult = mp.tasks.vision.FaceLandmarkerResult

#hand landmarking
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult

#get the path to the necessary helper files
ros_path = get_package_prefix("cv_basics")
ros_path = ros_path[:ros_path.index("install")]
cv_path = ros_path + "src/cv_basics/cv_basics/"

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.declare_parameter("camera_topic", "camera_0")
        camera_topic = self.get_parameter("camera_topic").value

        self.declare_parameter("display", False)
        self.display = self.get_parameter("display").value

        self.declare_parameter("detect_faces", False)
        self.declare_parameter("landmark_faces", False)
        self.declare_parameter("landmark_hands", False)

        self.detect_faces = self.get_parameter("detect_faces").value
        self.landmark_faces = self.get_parameter("landmark_faces").value
        self.landmark_hands = self.get_parameter("landmark_hands").value

        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.generic_callback,
            10
        )

        if self.detect_faces:
            self.face_detector_options = FaceDetectorOptions(
                base_options = BaseOptions(model_asset_path=(cv_path + "models/blaze_face_short_range.tflite")),
                running_mode = VisionRunningMode.LIVE_STREAM,
                result_callback = self.update_face_detector_result
            )
            self.face_detection_publisher = self.create_publisher(FaceDetectionArray, "face_detection_results", 10)
            self.face_detector_result = None
            self.face_detector_helper = FaceDetector.create_from_options(self.face_detector_options)
        
        if self.landmark_faces:
            self.declare_parameter("num_faces", 1)
            num_faces = self.get_parameter("num_faces").value

            self.face_landmarker_options = FaceLandmarkerOptions(
                base_options = BaseOptions(model_asset_path=(cv_path + "models/face_landmarker.task")),
                running_mode = VisionRunningMode.LIVE_STREAM,
                result_callback = self.update_face_landmarker_result,
                num_faces = num_faces
            )
            self.face_landmark_publisher = self.create_publisher(FaceLandmarkArrayArray, "face_landmark_results", 10)
            self.face_landmarker_result = None
            self.face_landmarker_helper = FaceLandmarker.create_from_options(self.face_landmarker_options)

        if self.landmark_hands:
            self.declare_parameter("num_hands", 1)
            num_hands = self.get_parameter("num_hands").value

            self.hand_landmarker_options = HandLandmarkerOptions(
                base_options = BaseOptions(model_asset_path=(cv_path + "models/hand_landmarker.task")),
                running_mode = VisionRunningMode.LIVE_STREAM,
                result_callback = self.update_hand_landmarker_result,
                num_hands = num_hands
            )
            self.hand_landmark_publisher = self.create_publisher(HandArray, "hand_landmark_results", 10)
            self.hand_landmarker_result = None
            self.hand_landmarker_helper = HandLandmarker.create_from_options(self.hand_landmarker_options)

        self.br = CvBridge()

    def update_face_detector_result(self, result: FaceDetectorResult, output_image: mp.Image, timestamp_ms: int):
        self.face_detector_result = result

    def update_face_landmarker_result(self, result: FaceLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        self.face_landmarker_result = result

    def update_hand_landmarker_result(self, result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        self.hand_landmarker_result = result

    def generic_callback(self, data):
        #self.get_logger().info("Receiving video frame")
        self.current_time = int(time.time() * 1000)

        if self.detect_faces:
            self.face_detector(data)

        if self.landmark_faces:
            self.face_landmarker(data)

        if self.landmark_hands:
            self.hand_landmarker(data)

    def face_detector(self, data):
        currentFrame = self.br.imgmsg_to_cv2(data)

        image = mp.Image(image_format = mp.ImageFormat.SRGB, data=currentFrame)
        self.face_detector_helper.detect_async(image, self.current_time)

        detection_array_message = FaceDetectionArray()
        detection_array = []

        finalImage = currentFrame
        if self.face_detector_result != None:
            for detection in self.face_detector_result.detections:
                box = detection.bounding_box
                x = box.origin_x
                y = box.origin_y
                w = box.width
                h = box.height
                score = detection.categories[0].score

                detection_message = FaceDetection(x = x, y = y, w = w, h = h, score = score)
                detection_array.append(detection_message)

                if self.display:
                    cv2.rectangle(finalImage, (x,y), (x+w,y+h), (0, 255, 0), 4)

        detection_array_message.detections = detection_array
        self.face_detection_publisher.publish(detection_array_message)

        if self.display and not (self.landmark_faces or self.landmark_hands):
            cv2.imshow("camera", finalImage)
            cv2.waitKey(1)

    def face_landmarker(self, data):
        currentFrame = self.br.imgmsg_to_cv2(data)

        image = mp.Image(image_format = mp.ImageFormat.SRGB, data=currentFrame)
        self.face_landmarker_helper.detect_async(image, self.current_time)

        face_array_message = FaceLandmarkArrayArray()
        face_array = []

        finalImage = currentFrame
        if self.face_landmarker_result != None and len(self.face_landmarker_result.face_landmarks) != 0:
            for landmarkList in self.face_landmarker_result.face_landmarks:
                landmark_array_message = FaceLandmarkArray()
                landmark_array = []
                for landmark in landmarkList:
                    landmark_message = Landmark(x = landmark.x, y = landmark.y, z = landmark.z)
                    landmark_array.append(landmark_message)
                    #print(landmark.x, landmark.y, landmark.z)
                landmark_array_message.landmarks = landmark_array
                face_array.append(landmark_array_message)

                if self.display:
                    normalizedLandmarks = landmark_pb2.NormalizedLandmarkList()
                    normalizedLandmarks.landmark.extend([
                        landmark_pb2.NormalizedLandmark(x = landmark.x, y = landmark.y, z = landmark.z) for landmark in landmarkList
                    ])

                    solutions.drawing_utils.draw_landmarks(
                        finalImage,
                        normalizedLandmarks,
                        solutions.face_mesh.FACEMESH_TESSELATION,
                        None,
                        solutions.drawing_styles.get_default_face_mesh_tesselation_style()
                    )

        face_array_message.faces = face_array
        self.face_landmark_publisher.publish(face_array_message)
            
        if self.display and not self.landmark_hands:
            cv2.imshow("camera", finalImage)
            cv2.waitKey(1)

    def hand_landmarker(self, data):
        currentFrame = self.br.imgmsg_to_cv2(data)

        image = mp.Image(image_format = mp.ImageFormat.SRGB, data=currentFrame)
        self.hand_landmarker_helper.detect_async(image, self.current_time)

        hand_array = []

        finalImage = currentFrame
        if self.hand_landmarker_result != None and self.hand_landmarker_result.hand_landmarks != []:
            for landmarkList in self.hand_landmarker_result.hand_landmarks: #iterate through each hand
                hand_landmarks = []
                for landmark in landmarkList:
                    landmark_message = Landmark(x = landmark.x, y = landmark.y, z = landmark.z)
                    hand_landmarks.append(landmark_message)
                
                hand_message = Hand(landmarks = hand_landmarks)
                hand_array.append(hand_message)

                if self.display:
                    #draw landmarks
                    normalizedLandmarks = landmark_pb2.NormalizedLandmarkList()
                    normalizedLandmarks.landmark.extend([
                        landmark_pb2.NormalizedLandmark(x = landmark.x, y = landmark.y, z = landmark.z) for landmark in landmarkList
                    ])

                    solutions.drawing_utils.draw_landmarks(
                        finalImage,
                        normalizedLandmarks,
                        solutions.hands.HAND_CONNECTIONS,
                        solutions.drawing_styles.get_default_hand_landmarks_style(),
                        solutions.drawing_styles.get_default_hand_connections_style()
                    )

        hand_array_mesage = HandArray(hands = hand_array)
        self.hand_landmark_publisher.publish(hand_array_mesage)

        if self.display:
            cv2.imshow("camera", finalImage)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    imageSubscriber = ImageSubscriber()
    rclpy.spin(imageSubscriber)
    imageSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
