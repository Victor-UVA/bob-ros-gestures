#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from moviepy.editor import VideoFileClip
import pygame
import os


def callback(data):
    if data.data is True:
        pygame.init()

        # Set the screen dimensions to 9:16 aspect ratio (portrait)
        screen_width, screen_height = 1080, 1920  # You can adjust these dimensions

        # Load the video file
        video_file = "/home/andrew/Downloads/ThomasJeffersonThornton.mp4"
        clip = VideoFileClip(video_file)

        # Calculate the aspect-ratio-preserving size for the video
        video_width, video_height = clip.size
        video_aspect_ratio = video_width / video_height

        if video_aspect_ratio > 9 / 16:
            video_width = int(screen_width)
            video_height = int(screen_width / video_aspect_ratio)
        else:
            video_height = int(screen_height)
            video_width = int(screen_height * video_aspect_ratio)

        # Create a Pygame screen with the specified dimensions
        screen = pygame.display.set_mode((screen_width, screen_height))

        # Set the clip size to match the video's new size
        clip = clip.resize((video_width, video_height))

        # Rotate the video by 90 degrees counterclockwise
        clip = clip.rotate(90)

        # Hide the mouse cursor
        pygame.mouse.set_visible(False)

        # Create a Pygame clock to control playback speed
        clock = pygame.time.Clock()

        # Play the video with audio
        clip.preview(fps=15)

        pygame.mouse.set_visible(True)

        # Close both Pygame windows
        pygame.quit()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/start_video', Bool, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
