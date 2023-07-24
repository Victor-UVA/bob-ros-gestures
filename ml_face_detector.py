import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

BaseOptions = mp.tasks.BaseOptions
FaceDetector = mp.tasks.vision.FaceDetector
FaceDetectorOptions = mp.tasks.vision.FaceDetectorOptions
FaceDetectorResult = mp.tasks.vision.FaceDetectorResult
VisionRunningMode = mp.tasks.vision.RunningMode

lastResult = None
def showResult(result: FaceDetectorResult, output_image: mp.Image, timestamp_ms: int):
    global lastResult
    lastResult = result

options = FaceDetectorOptions(
    base_options = BaseOptions(model_asset_path="blaze_face_short_range.tflite"),
    running_mode = VisionRunningMode.LIVE_STREAM,
    result_callback = showResult
)

capture = cv2.VideoCapture(0)
with FaceDetector.create_from_options(options) as detector:
    while True:
        ret, frame = capture.read()
        time = int(capture.get(cv2.CAP_PROP_POS_MSEC))
        mp_image = mp.Image(image_format = mp.ImageFormat.SRGB, data=frame)
        detector.detect_async(mp_image, time)

        finalImage = frame
        if lastResult != None:
            for detection in lastResult.detections:
                box = detection.bounding_box
                x = box.origin_x
                y = box.origin_y
                w = box.width
                h = box.height
                cv2.rectangle(finalImage, (x,y), (x+w,y+h), (0, 255, 0), 4)
        cv2.imshow("camera", finalImage)
        cv2.waitKey(1)