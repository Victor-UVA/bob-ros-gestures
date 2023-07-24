# haar_face_detector.py>
import cv2

faceClassifier = cv2.CascadeClassifier(
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
)

#detects any faces in an image and draws a green bounding box around them
def detectFace(image):
    grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    face = faceClassifier.detectMultiScale(
        grayImage,
        scaleFactor = 1.1,
        minNeighbors = 9,
        minSize = (40, 40)
    )
    for (x, y, w, h) in face:
        cv2.rectangle(image, (x,y), (x+w,y+h), (0, 255, 0), 4)

    return image