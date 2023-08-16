#!/usr/bin/env python

import rospy
import math
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool


class MoveBaseSeq():

    def __init__(self):
        rospy.init_node('move_goal')

        self.finished = Bool()
        self.finished = False
        # self.origin_checker = False

        self.is_finished = rospy.Publisher('/first_finished', Bool, queue_size=10)
        self.done_waving = rospy.Subscriber('/is_ready_to_move', Bool, self.ready_to_move)


    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt + 1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.finished = False
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose " + str(
                self.goal_cnt) + " received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached")
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("First goal pose reached!")
                self.finished = True
            msg = Bool()
            msg.data = self.finished
            self.is_finished.publish(msg)
                # return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_cnt) + " aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_cnt) + " rejected, shutting down!")
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

    def ready_to_move(self, data):
        print("HERE IT IS:", data.data)
        if data.data is True:
            points_seq = rospy.get_param('move_goal/p_seq')
            # Only yaw angle required (no rotations around x and y axes) in deg:
            yaweulerangles_seq = rospy.get_param('move_goal/yea_seq')
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


if __name__ == '__main__':

    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation has finished.")
    rospy.spin()
