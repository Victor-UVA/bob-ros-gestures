#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import cv2

def callback(data):
    if data.data is True:
        file_name = "/home/andrew/Downloads/test2.mp4"
        window_name = "window"
        interframe_wait_ms = 30

        cap = cv2.VideoCapture(file_name)
        if not cap.isOpened():
            print("Error: Could not open video.")
            exit()

        cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        while (True):
            ret, frame = cap.read()
            if not ret:
                print("Reached end of video, exiting.")
                break

            cv2.imshow(window_name, frame)
            if cv2.waitKey(interframe_wait_ms) & 0x7F == ord('q'):
                print("Exit requested.")
                break

        cap.release()
        cv2.destroyAllWindows()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/start_video', Bool, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
