import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions
from enum import Enum

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

lastResult = None
def updateResults(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global lastResult
    lastResult = result

#initialize hand landmarker options
options = HandLandmarkerOptions(
    base_options = BaseOptions(model_asset_path="../models/hand_landmarker.task"),
    running_mode = VisionRunningMode.LIVE_STREAM,
    result_callback = updateResults
)

#helper function for creating the hand bounding box
def transformByScale(coord, center, scale):
    return int((coord - center) * scale + center)

#find coordinates for the hand bounding box
def getHandBoxCoords(landmarkList):
    xList = [landmark.x for landmark in landmarkList]
    yList = [landmark.y for landmark in landmarkList]
    height, width, _ = finalImage.shape #get image dimensions
    xMin = min(xList) * width
    xMax = max(xList) * width
    yMin = min(yList) * height
    yMax = max(yList) * height

    #processing (make the box a bit bigger in all directions)
    scale = 1.2
    xCenter = (xMin + xMax)/2
    yCenter = (yMin + yMax)/2
    xMin = transformByScale(xMin, xCenter, scale)
    xMax = transformByScale(xMax, xCenter, scale)
    yMin = transformByScale(yMin, yCenter, scale)
    yMax = transformByScale(yMax, yCenter, scale)

    return (xMin,yMin), (xMax,yMax)

#rough hand orientation
class HandOrientation(Enum):
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4

#get rough hand orientation from landmarks
def calculateHandOrientation(landmarkList):
    point0 = landmarkList[0]
    point9 = landmarkList[9]
    xDelta = point9.x - point0.x
    yDelta = point9.y - point0.y
    ratio = abs(yDelta / xDelta)

    if ratio >= 1:
        if yDelta <= 0:
            orientation = HandOrientation.UP
        else:
            orientation = HandOrientation.DOWN
    else:
        if xDelta >= 0:
            orientation = HandOrientation.LEFT
        else:
            orientation = HandOrientation.RIGHT

    return orientation

#finger position
class FingerPosition(Enum):
    OPEN = 1
    CLOSED = 2

#calculate finger position for a given finger
def calculateFingerPosition(basePoint, closePoint, farPoint):
    xDeltaClose = closePoint.x - basePoint.x
    yDeltaClose = closePoint.y - basePoint.y
    distanceClose = (xDeltaClose**2 + yDeltaClose**2)**0.5

    xDeltaFar = farPoint.x - basePoint.x
    yDeltaFar = farPoint.y - basePoint.y
    distanceFar = (xDeltaFar**2 + yDeltaFar**2)**0.5

    if distanceClose >= distanceFar:
        return FingerPosition.CLOSED
    else:
        return FingerPosition.OPEN

#calculate finger positions for all fingers
def calculateFingerPositions(landmarkList):
    basePoint = landmarkList[0]

    thumbBase = landmarkList[17]
    thumbClose = landmarkList[1]
    thumbFar = landmarkList[4]
    thumbPosition = calculateFingerPosition(thumbBase, thumbClose, thumbFar)

    indexClose = landmarkList[5]
    indexFar = landmarkList[8]
    indexPosition = calculateFingerPosition(basePoint, indexClose, indexFar)

    middleClose = landmarkList[9]
    middleFar = landmarkList[12]
    middlePosition = calculateFingerPosition(basePoint, middleClose, middleFar)

    ringClose = landmarkList[13]
    ringFar = landmarkList[16]
    ringPosition = calculateFingerPosition(basePoint, ringClose,ringFar)

    littleClose = landmarkList[17]
    littleFar = landmarkList[20]
    littlePosition = calculateFingerPosition(basePoint, littleClose, littleFar)

    return (littlePosition, ringPosition, middlePosition, indexPosition, thumbPosition)

#number of raised fingers
class Count(Enum):
    ZERO = 0
    ONE = 1
    TWO = 2
    THREE = 3
    FOUR = 4
    FIVE = 5
    UNKNOWN = 6

#what hand positions correspond to what finger counts
countDict = {
    (FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN): Count.FIVE,

    (FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.CLOSED): Count.FOUR,
    (FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.CLOSED, FingerPosition.CLOSED): Count.THREE,
    (FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED): Count.TWO,
    (FingerPosition.OPEN, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED): Count.ONE,

    (FingerPosition.CLOSED, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.CLOSED): Count.THREE,
    (FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.CLOSED): Count.TWO,
    (FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.OPEN, FingerPosition.CLOSED): Count.ONE,

    (FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.OPEN, FingerPosition.OPEN, FingerPosition.OPEN): Count.THREE,
    (FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.OPEN, FingerPosition.OPEN): Count.TWO,
    (FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.OPEN): Count.ONE,

    (FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED, FingerPosition.CLOSED): Count.ZERO
}

#match finger positions to count
def matchFingerPositionsToCount(fingerPositionProfile):
    try:
        count = countDict[fingerPositionProfile]
    except:
        count = Count.UNKNOWN
    print(count)

#thumb orientation
class ThumbOrientation(Enum):
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4

#get orientation for the thumb specifically
def calculateThumbOrientation(landmarkList):
    point1 = landmarkList[1]
    point4 = landmarkList[4]
    xDelta = point4.x - point1.x
    yDelta = point4.y - point1.y
    ratio = abs(yDelta / xDelta)

    if ratio >= 1:
        if yDelta <= 0:
            orientation = ThumbOrientation.UP
        else:
            orientation = ThumbOrientation.DOWN
    else:
        if xDelta >= 0:
            orientation = ThumbOrientation.LEFT
        else:
            orientation = ThumbOrientation.RIGHT

    return orientation

#general gesture matching
class Gesture(Enum):
    THUMBS_UP = 1
    THUMBS_DOWN = 2
    PALM = 3
    FIST = 4
    UNKNOWN = 5

#match gesture using some basic logic
def matchGesture(landmarkList):
    fingerPositionProfile = calculateFingerPositions(landmarkList)
    fingerCount = sum([1 if x == FingerPosition.OPEN else 0 for x in fingerPositionProfile])
    thumbOpen = (fingerPositionProfile[4] == FingerPosition.OPEN)
    print(thumbOpen)
    thumbOrientation = calculateThumbOrientation(landmarkList)

    gesture = Gesture.UNKNOWN
    if fingerCount == 0:
        gesture = Gesture.FIST
    elif fingerCount == 5:
        gesture = Gesture.PALM
    elif fingerCount == 1 and thumbOpen:
        if thumbOrientation == ThumbOrientation.UP:
            gesture = Gesture.THUMBS_UP
        elif thumbOrientation == ThumbOrientation.DOWN:
            gesture = Gesture.THUMBS_DOWN

    return gesture

capture = cv2.VideoCapture(0) #get video source
with HandLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = capture.read() #get frame
        time = int(capture.get(cv2.CAP_PROP_POS_MSEC)) #get time
        print("time", time)
        mp_image = mp.Image(image_format = mp.ImageFormat.SRGB, data=frame) #transform into MediaPipe image format
        landmarker.detect_async(mp_image, time) #detect hand landmarks in image

        finalImage = frame
        if lastResult != None and lastResult.hand_landmarks != []:
            landmarkList = lastResult.hand_landmarks[0]

            #draw bounding box
            minCoord, maxCoord = getHandBoxCoords(landmarkList)
            cv2.rectangle(finalImage, minCoord, maxCoord, (0, 255, 0), 4)

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

            #print(calculateThumbOrientation(landmarkList))
            #fingerPositionProfile = calculateFingerPositions(landmarkList)
            #matchFingerPositionsToCount(fingerPositionProfile)
            print(matchGesture(landmarkList))

        cv2.imshow("camera", finalImage)
        cv2.waitKey(1)