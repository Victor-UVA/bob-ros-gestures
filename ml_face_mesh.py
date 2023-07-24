import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions
from enum import Enum
import numpy as np

BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
FaceLandmarkerResult = mp.tasks.vision.FaceLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

lastResult = None
def showResult(result: FaceLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global lastResult
    lastResult = result

options = FaceLandmarkerOptions(
    base_options = BaseOptions(model_asset_path="face_landmarker.task"),
    running_mode = VisionRunningMode.LIVE_STREAM,
    result_callback = showResult
)

class FaceOrientation(Enum):
    LEFT = 1
    RIGHT = 2
    CENTER = 3

def getRoughFaceOrientation(landmarkList):
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

def getPreciseFaceOrientation(landmarkList, image):
    height, width, _ = image.shape
    face2d = []
    face3d = []
    for landmarkID in [33, 263, 1, 61, 291, 199]:
        landmark = landmarkList[landmarkID]
        x = int(landmark.x * width)
        y = int(landmark.y * height)
        face2d.append([x, y])
        face3d.append([x, y, landmark.z])
    face2dNP = np.array(face2d, dtype=np.float64)
    face3dNP = np.array(face3d, dtype=np.float64)

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

    print("pitch", pitch)
    print("yaw  ", yaw)

capture = cv2.VideoCapture(0)
with FaceLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = capture.read()
        time = int(capture.get(cv2.CAP_PROP_POS_MSEC))
        mp_image = mp.Image(image_format = mp.ImageFormat.SRGB, data=frame)
        landmarker.detect_async(mp_image, time)

        finalImage = frame
        if lastResult != None and lastResult.face_landmarks != []:
            landmarkList = lastResult.face_landmarks[0]
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

            #print(getRoughFaceOrientation(landmarkList))
            getPreciseFaceOrientation(landmarkList, finalImage)
            
        cv2.imshow("camera", finalImage)
        cv2.waitKey(1)