#!/usr/bin/env python3

import rospy
import math
import sys
import moveit_commander
# Brings in the SimpleActionClient
import actionlib
import time
import cv2
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from moviepy.editor import VideoFileClip
import pygame
import os


moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander
rarm_group = moveit_commander.MoveGroupCommander('rarm')
larm_group = moveit_commander.MoveGroupCommander('larm')
rarm_values = rarm_group.get_current_joint_values()
larm_values = larm_group.get_current_joint_values()

class MoveBaseSecondSeq():

    def __init__(self):

        self.checker = False
        rospy.Subscriber('/first_finished', Bool, self.callback)

    def start(self):
        points_seq = rospy.get_param('second_goal/p_seq2')
        # Only yaw angle required (no rotations around x and y axes) in deg:
        yaweulerangles_seq = rospy.get_param('second_goal/yea_seq2')
        # List of goal quaternions:
        quat_seq = list()
        # List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0

        for yawangle in yaweulerangles_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle * math.pi / 180, axes='sxyz'))))

        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i + n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        for point in points:
            # Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point), quat_seq[n - 3]))
            n += 1
        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        # wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to the move base server")
        rospy.loginfo("Starting goals achievements...")
        self.movebase_client()

    def callback(self, data):
        print(data.data, "is what we got over here!")
        if data.data is True:
            arms_up()
            video()
            time.sleep(5)
            arms_down()
            self.checker = True
            print("Starting the second node!")
            self.start()


    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt + 2)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+2)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose " + str(
                self.goal_cnt + 1) + " received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose " + str(self.goal_cnt+1) + " reached")
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                # rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.goal_cnt+1) + " was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_cnt+1) + " aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.goal_cnt+1) + " has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_cnt+1) + " rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose " + str(
                self.goal_cnt) + " received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        # rospy.spin()


def arms_up():
        # Make sure arms are set down
        rarm_values[0] = 0
        rarm_values[1] = 0
        rarm_values[2] = 0
        rarm_values[3] = 0
        rarm_values[7] = 0
        rarm_group.set_joint_value_target(rarm_values)
        rarm_group.set_max_velocity_scaling_factor(1)

        larm_values[0] = 0
        larm_values[1] = 0
        larm_values[2] = 0
        larm_values[3] = 0
        larm_values[7] = 0
        larm_group.set_joint_value_target(larm_values)
        larm_group.set_max_velocity_scaling_factor(1)

        plan = rarm_group.plan()
        plan2 = larm_group.plan()
        rarm_group.go(wait=True)
        larm_group.go(wait=True)

        # Pick arm up and put it in gesture location
        rarm_values[0] = -0.354
        rarm_values[1] = 0.0
        rarm_values[2] = 0.9734
        rarm_values[3] = -1.32
        rarm_values[7] = -0.87895
        rarm_group.set_joint_value_target(rarm_values)

        plan3 = rarm_group.plan()
        rarm_group.go(wait=True)

        larm_values[0] = 0.0
        larm_values[1] = 0.0
        larm_values[2] = 0.9734
        larm_values[3] = -1.5
        larm_values[7] = 1.539
        larm_group.set_joint_value_target(larm_values)

        plan4 = larm_group.plan()
        larm_group.go(wait=True)


def arms_down():
        # Put arms back
        rarm_values[0] = 0
        rarm_values[1] = 0
        rarm_values[2] = 0
        rarm_values[3] = 0
        rarm_values[7] = 0
        rarm_group.set_joint_value_target(rarm_values)

        plan5 = rarm_group.plan()
        rarm_group.go(wait=True)

        larm_values[0] = 0
        larm_values[1] = 0
        larm_values[2] = 0
        larm_values[3] = 0
        larm_values[7] = 0
        larm_group.set_joint_value_target(larm_values)

        plan6 = larm_group.plan()
        larm_group.go(wait=True)


def video():
    pygame.init()

    # Set the screen dimensions to 9:16 aspect ratio (portrait)
    screen_width, screen_height = 1080, 1920  # You can adjust these dimensions

    # Load the video file
    video_file = "/home/andrew/Downloads/ThomasJefferson2.mp4"
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
    clip.preview(fps=10)

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
    # waiting_for_key = True
    # while waiting_for_key:
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
    #             waiting_for_key = False

    # Restore the mouse cursor visibility
    pygame.mouse.set_visible(True)

    # Close both Pygame windows
    pygame.quit()


def main():
    rospy.init_node('move_base_return_sequence')
    # arms_up()
    # arms_down()
    MoveBaseSecondSeq()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation has finished.")
    # moveit_commander.roscpp_shutdown