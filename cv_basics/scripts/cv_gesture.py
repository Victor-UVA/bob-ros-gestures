#!/usr/bin/env python

from multiprocessing.connection import wait
import sys
import rospy
import moveit_commander
import time
import moveit_msgs.msg
import numpy
import move_goal

from std_msgs.msg import Bool
from cv_basics.msg import FaceDetection
from cv_basics.msg import FaceDetectionArray
 
latch = False
latch2 = False
group_latch = False
waved = False
msg = False
face_detections = [0] * 60
# group_detections = [0] * 60

moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('move_seed_wave')

robot = moveit_commander.RobotCommander
rarm_group = moveit_commander.MoveGroupCommander('rarm')
larm_group = moveit_commander.MoveGroupCommander('larm')
rhand_group = moveit_commander.MoveGroupCommander('rhand')
lhand_group = moveit_commander.MoveGroupCommander('lhand')
w_group = moveit_commander.MoveGroupCommander("waist")
# rospy.Publisher('/move_group/display_planned_path',
# moveit_msgs.msg.DisplayTrajectory, queue_size=10)

rarm_values = rarm_group.get_current_joint_values()
larm_values = larm_group.get_current_joint_values()
lhand_values = lhand_group.get_current_joint_values()
rhand_values = rhand_group.get_current_joint_values()
waist_values = w_group.get_current_joint_values()

pub = rospy.Publisher('/is_ready_to_move', Bool, queue_size=10)


def gesture(detections):

    global latch
    global latch2
    global group_latch
    global face_detections
    global waved

    group_latch = False

    for i in range(58, -1, -1):
        face_detections[i+1] = face_detections[i]
        # group_detections[i+1] = group_detections[i]

    face_detections[0] = 0
    # group_detections[0] = 0
    # print("length is: ", len(detections.detections))
    for face in detections.detections:
        # print(face.score, latch)
        if face.score >= .60:
            face_detections[0] = 1
            # if group_latch:
            #     group_detections[0] = 1
            # else:
            #     group_latch = True

    face_detection_percentage = sum(face_detections)/60
    # group_detection_percentage = sum(group_detections)/30

    # print("Face Detections:", face_detection_percentage)
    # print("Group Detections:", group_detection_percentage)

    if face_detection_percentage >= 0.8 and not latch:
        print('waving!')
        gesture_wave()
        waved = True
        latch = True
    elif face_detection_percentage == 0.0:
        if waved:
            time.sleep(60)
            waved = False
        print('reset')
        latch = False

    # if len(detections.detections) >= 2 and group_detection_percentage >= 0.8 and not latch2:
    #     print('bowing!')
    #     gesture_bow()
    #     latch2 = True
    # elif group_detection_percentage == 0.0:
    #     print('group reset')
    #     latch2 = False


def gesture_wave():
    global msg

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

    # Pick arm up and put it in wave location
    rarm_values[0] = -0.5
    # rarm_values[1] = -0.11
    # rarm_values[2] = 0.228
    rarm_values[3] = -2.1
    rarm_values[7] = 1.5
    rarm_group.set_joint_value_target(rarm_values)

    plan3 = rarm_group.plan()
    rarm_group.go(wait=True)

    # Open hand up
    rhand_values[0] = 1
    rhand_group.set_joint_value_target(rhand_values)

    plan4 = rhand_group.plan()
    rhand_group.go(wait=True)

    # Wave
    rarm_values[2] = 0.2
    rarm_group.set_joint_value_target(rarm_values)

    plan4 = rarm_group.plan()
    rarm_group.go(wait=True)

    rarm_values[2] = 0
    rarm_group.set_joint_value_target(rarm_values)

    plan5 = rarm_group.plan()
    rarm_group.go(wait=True)

    rarm_values[2] = 0.2
    rarm_group.set_joint_value_target(rarm_values)

    plan6 = rarm_group.plan()
    rarm_group.go(wait=True)

    # rarm_values[2] = 0
    # rarm_group.set_joint_value_target(rarm_values)

    # plan7 = rarm_group.plan()
    # rarm_group.go(wait=True)

    # Put arms back
    rarm_values[0] = 0
    rarm_values[1] = 0
    rarm_values[2] = 0
    rarm_values[3] = 0
    rarm_values[7] = 0
    rarm_group.set_joint_value_target(rarm_values)

    plan10 = rarm_group.plan()
    rarm_group.go(wait=True)

    msg = True
    pub.publish(msg)

def gesture_bow():
    # Set arms down
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

    plan3 = rarm_group.plan()
    plan4 = larm_group.plan()
    rarm_group.go(wait=True)
    larm_group.go(wait=True)

    # Bow down
    waist_values[0] = 0
    waist_values[1] = 0.61
    waist_values[2] = 0
    w_group.set_joint_value_target(waist_values)
    w_group.set_max_velocity_scaling_factor(1)

    plan2 = w_group.plan()
    w_group.go(wait=True)

    # Come up
    waist_values[0] = 0
    waist_values[1] = 0
    waist_values[2] = 0
    w_group.set_joint_value_target(waist_values)

    plan2 = w_group.plan()
    w_group.go(wait=True)


def subscriber_node():

    rospy.init_node("cv_gesture")
    rospy.Subscriber('/face_detection_results', FaceDetectionArray, gesture, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber_node()
    except rospy.ROSInterruptException:
        pass

    moveit_commander.roscpp_shutdown
