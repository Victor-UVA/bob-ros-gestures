# Bob ROS Gestures
Hand and face detection code to use for Bob ROS. Code is based off of ROS 2 but has been translated into ROS.

Creators (ask us if you need help): Evan Conway (original ROS 2 code) and Andrew Vithoulkas (conversion to ROS and continued development)

Core Files
- publisher.py: Publishes camera to a ROS topic (parameters: source)
  - Parameters:
      - source (int): what camera source to publish (defaults to 0)
  - Topics:
    - Output:
      - /camera_[x] (where [x] = source)
- subscriber.py: Reads camera data from topic and scans for hands / faces depending on the specified parameters
  - Parameters:
    - camera_topic (string): the camera topic to listen to (defaults to "camera_0")
    - display (bool): whether or not to visually display the results (defaults to false)
    - detect_faces (bool): toggles basic face detection (defaults to false)
    - landmark_faces (bool): toggles face landmarking (defaults to false)
    - landmark_hands (bool): toggles hand landmarking (defaults to false)
  - Topics:
    - Input:
      - /[camera_topic] (depends on what the parameter camera_topic is)
    - Output:
      - /face_detection_results
      - /face_landmark_results
      - /hand_landmark_results

Model Files:
- blaze_face_short_range.tflite: Data for short range face detection
- face_landmarker.task: Data for face landmarking
- hand_landmarker.task: Data for hand landmarking
