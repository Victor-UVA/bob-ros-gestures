#!/usr/bin/env python3

from moviepy.editor import VideoFileClip
import pygame
import os

screen_width, screen_height = 1080, 1920  # You can adjust these dimensions

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

image = pygame.transform.scale(image, (image_width, image_height))
image = pygame.transform.rotate(image, 90)

# Calculate the position to center the image on the screen
image_x = (screen_width - image_width) // 2
image_y = (screen_height - image_height) // 2

# Create a new Pygame window for the image
image_screen = pygame.display.set_mode((screen_width * (16/9), screen_height * (9/16)))
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


