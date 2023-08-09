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


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('move_goal/p_seq')
        yaweulerangles_seq = rospy.get_param('move_goal/yea_seq')
        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0

        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))

        n = 3
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n-3]))
            n += 1
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # wait = self.client.wait_for_server(rospy.Duration(5.0))
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to the move base server")
        rospy.loginfo("Starting goals achievements...")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt + 1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
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
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

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
        rospy.spin()

    # def movebase_client():
    #     # Create an action client called "move_base" with action definition file "MoveBaseAction"
    #     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    #
    #     # Waits until the action server has started up and started listening for goals.
    #     client.wait_for_server()
    #
    #     # Creates a new goal with the MoveBaseGoal constructor
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = "map"
    #     goal.target_pose.header.stamp = rospy.Time.now()
    #     # Move ___ meters forward along the x axis of the "map" coordinate frame
    #     goal.target_pose.pose.position.x = 0
    #     # Move ___ meters forward along the y axis
    #     goal.target_pose.pose.position.y = -2.5
    #     # ___ rotation of the mobile base frame w.r.t. map frame
    #     goal.target_pose.pose.orientation.w = 0.5
    #
    #     # Sends the goal to the action server.
    #     client.send_goal(goal)
    #     # Waits for the server to finish performing the action.
    #     wait = client.wait_for_result()
    #     # If the result doesn't arrive, assume the Server is not available
    #     if not wait:
    #         rospy.logerr("Action server not available!")
    #         rospy.signal_shutdown("Action server not available!")
    #     else:
    #         # Result of executing the action
    #         return client.get_result()
    #
    #     # If the python node is executed as main process (sourced directly)


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation has finished.")