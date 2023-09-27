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
        video_file = "/home/andrew/Downloads/ThomasJefferson1.mp4"
        clip = VideoFileClip(video_file)

        # Calculate the aspect-ratio-preserving size for the video
        video_width, video_height = clip.size
        video_aspect_ratio = video_width / video_height

        if video_aspect_ratio > 9 / 16:
            video_width = int(screen_width) * 1.1
            video_height = int(screen_width / video_aspect_ratio) * 1.2
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

        # When the video ends, open a new window with an image
        image_file = "/home/andrew/Downloads/jefferson.png"  # Change to the path of your image file
        image = pygame.image.load(image_file)

        # Resize the image to fit within the screen dimensions
        image_width, image_height = image.get_size()
        image_aspect_ratio = image_width / image_height

        if image_aspect_ratio > 9 / 16:
            image_width = int(screen_width)
            image_height = int(screen_width / image_aspect_ratio)
        else:
            image_height = int(screen_height)
            image_width = int(screen_height * image_aspect_ratio)

        image = pygame.transform.scale(image, (image_width * 0.8, image_height * 0.8))
        image = pygame.transform.rotate(image, 90)

        # Calculate the position to center the image on the screen
        image_x = (screen_width - image_width) // 2
        image_y = (screen_height - image_height) // 2

        # Create a new Pygame window for the image
        image_screen = pygame.display.set_mode((screen_width * 2, screen_height / 1.8))
        image_screen.blit(image, (image_x, image_y))
        pygame.display.flip()

        # Wait for user input to exit the image window
        waiting_for_key = True
        while waiting_for_key:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    waiting_for_key = False

        # Restore the mouse cursor visibility
        pygame.mouse.set_visible(True)

        # Close both Pygame windows
        pygame.quit()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/start_video', Bool, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
