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
from enum import Enum
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
FaceLandmarkerResult = mp.tasks.vision.FaceLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

class FaceOrientation(Enum):
    LEFT = 1
    RIGHT = 2
    CENTER = 3

#this is a ROS node based on code from ml_face_detector.py
class FaceMeshTransformer(Node):
    def __init__(self):
        super().__init__("face_mesh_transformer")
        self.colorSubscription = self.create_subscription(Image, "camera/color/image_raw", self.colorCallback, 10)

        self.depthSubscription = self.create_subscription(Image, "camera/aligned_depth_to_color/image_raw", self.depthCallback, 10)

        self.pub_face_pitch = self.create_publisher(Float64, "face_pitch", 10)
        self.pub_face_yaw = self.create_publisher(Float64, "face_yaw", 10)
        self.pub_nose_2d = self.create_publisher(Float64MultiArray, "nose_2d", 10)
        self.pub_nose_depth = self.create_publisher(Float64, "nose_depth", 10)

        self.options = FaceLandmarkerOptions(
            base_options = BaseOptions(model_asset_path="../models/face_landmarker.task"),
            running_mode = VisionRunningMode.IMAGE
        )
        
        self.br = CvBridge()

        self.nose2d = [0, 0]
        self.noseDepth = 0

    def getRoughFaceOrientation(self, landmarkList):
        leftPoint = landmarkList[234]
        rightPoint = landmarkList[454]
        zDelta = rightPoint.z - leftPoint.z
        print(zDelta)

        orientation = FaceOrientation.CENTER
        if abs(zDelta) >= 0.08:
            if zDelta <= 0:
                orientation = FaceOrientation.RIGHT
            else:
                orientation = FaceOrientation.LEFT
        return orientation

    def getPreciseFaceOrientation(self, landmarkList, image):
        height, width, _ = image.shape
        face2d = []
        face3d = []
        for landmarkID in [1, 33, 263, 61, 291, 199]:
            landmark = landmarkList[landmarkID]
            x = int(landmark.x * width)
            y = int(landmark.y * height)
            face2d.append([x, y])
            face3d.append([x, y, landmark.z])
        face2dNP = np.array(face2d, dtype=np.float64)
        face3dNP = np.array(face3d, dtype=np.float64)
        self.nose2d = face2d[0]

        cameraMatrix = np.array([
            [width, 0, height/2],
            [0, width, width/2],
            [0, 0, 1]
        ])

        distanceMatrix = np.zeros((4, 1), dtype=np.float64)
        successs, rotationVector, translationVector = cv2.solvePnP(face3dNP, face2dNP, cameraMatrix, distanceMatrix)
        rotationMatrix, _ = cv2.Rodrigues(rotationVector)
        angles = cv2.RQDecomp3x3(rotationMatrix)[0]
        
        pitch = angles[0] * 360
        yaw = angles[1] * 360

        return pitch, yaw

    def colorCallback(self, data):
        self.get_logger().info("Receiving video frame")
        currentFrame = self.br.imgmsg_to_cv2(data, "bgr8")

        with FaceLandmarker.create_from_options(self.options) as landmarker:
            image = mp.Image(image_format = mp.ImageFormat.SRGB, data=currentFrame)
            result = landmarker.detect(image)

            finalImage = currentFrame
            if len(result.face_landmarks) != 0:
                landmarkList = result.face_landmarks[0]
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

                pitch, yaw = self.getPreciseFaceOrientation(landmarkList, finalImage)

                pitch_msg = Float64()
                pitch_msg.data = pitch
                self.pub_face_pitch.publish(pitch_msg)

                yaw_msg = Float64()
                yaw_msg.data = yaw
                self.pub_face_yaw.publish(yaw_msg)
            
        cv2.imshow("camera", finalImage)
        cv2.waitKey(1)

    def depthCallback(self, data):
        nose2d_msg = Float64MultiArray()
        nose2d_msg.data = [float(x) for x in self.nose2d]
        self.pub_nose_2d.publish(nose2d_msg)

        depthFrame = self.br.imgmsg_to_cv2(data, "16UC1")
        if depthFrame.shape[0] == 720 and depthFrame.shape[1] == 1280:
            noseDepth = depthFrame[self.nose2d[1]][self.nose2d[0]]
            nose_depth_msg = Float64()
            nose_depth_msg.data = float(noseDepth)
            self.pub_nose_depth.publish(nose_depth_msg)

def main(args=None):
    rclpy.init(args=args)
    faceMeshTransformer = FaceMeshTransformer()
    rclpy.spin(faceMeshTransformer)
    faceMeshTransformer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()