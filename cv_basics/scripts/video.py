#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from moviepy.editor import VideoFileClip
import pygame
import os

def callback(data):
    if data.data is True:
        # Initialize pygame
        pygame.init()

        # Get the screen dimensions
        screen_width, screen_height = pygame.display.Info().current_w / 2, pygame.display.Info().current_h

        # Load the video file
        video_file = "/home/andrew/Downloads/ThomasJefferson1.mp4"
        clip = VideoFileClip(video_file)

        # Create a Pygame screen in fullscreen mode without window frame
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN | pygame.NOFRAME)

        # Set the clip size to match the screen size
        clip = clip.resize((screen_width, screen_height))

        # Hide the mouse cursor
        pygame.mouse.set_visible(False)

        # Create a Pygame clock to control playback speed
        clock = pygame.time.Clock()

        # Play the video with audio
        clip.preview(fps=clip.fps)

        # Restore the mouse cursor visibility
        pygame.mouse.set_visible(True)

        # Close the Pygame window when the video ends
        pygame.quit()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/start_video', Bool, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
