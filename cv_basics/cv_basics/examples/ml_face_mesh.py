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

#initialize face landmarker options
options = FaceLandmarkerOptions(
    base_options = BaseOptions(model_asset_path="../models/face_landmarker.task"),
    running_mode = VisionRunningMode.LIVE_STREAM,
    result_callback = showResult
)

#create Enum of various face orientations
class FaceOrientation(Enum):
    LEFT = 1
    RIGHT = 2
    CENTER = 3

#use facial landmarks to estimate the approximate face orientation
def getRoughFaceOrientation(landmarkList):
    leftPoint = landmarkList[234] #get the 3d location of a point on the left side of the face
    rightPoint = landmarkList[454] #get the 3d location of the same point on the right side of the face
    zDelta = rightPoint.z - leftPoint.z #get the difference in closeness between the left and the right sides of the face
    print(zDelta)

    #classify face orientation based on relative distance
    if abs(zDelta) >= 0.08:
        if zDelta <= 0:
            orientation = FaceOrientation.RIGHT
        else:
            orientation = FaceOrientation.LEFT
    else:
        orientation = FaceOrientation.CENTER
    
    return orientation

#do some math to get the face pitch and yaw
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

capture = cv2.VideoCapture(0) #get video source
with FaceLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = capture.read() #get frame
        time = int(capture.get(cv2.CAP_PROP_POS_MSEC)) #get time
        mp_image = mp.Image(image_format = mp.ImageFormat.SRGB, data=frame) #transform into MediaPipe image format
        landmarker.detect_async(mp_image, time) #detect face landmarks in image

        finalImage = frame
        if lastResult != None and lastResult.face_landmarks != []: #draw all face landmarks
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