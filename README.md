# Bob ROS Gestures
Hand and face detection code to use for Bob ROS. Code is based off of ROS 2 but most should also be usable in ROS.

Creator (ask me if you need help): Evan Conway.

Code Files:
- face_mesh_transformer.py: ROS node based off of code in ml_face_mesh.py
- haar_face_detector: Face detector based off of Haar Cascades
- ml_face_detector.py: Face detector based off of Google MediaPipe
- ml_face_mesh.py: Detects face landmarks and estimates face pitch and yaw (based off of Google MediaPipe)
- ml_hand_detector.py: Detects hand landmarks and detects certain basic gestures
- publisher.py: Publishes camera to a ROS topic
- subscriber.py: ROS node based off of code in ml_face_detector.py

Other Files:
- blaze_face_short_range.tflite: Data for short range face detection
- face_landmarker.task: Data for face landmarking
- hand_landmarker.task: Data for hand landmarking
- testImage.jpg: Test image
