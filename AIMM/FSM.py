import rclpy
from rclpy.node import Node
import smach
import smach_ros
from std_msgs import String, Bool
from geographic_msgs.msg import Twist
from sensor_msgs.msg import Image
import threading
import time


class BehaviorTreesState(smach.State):
    '''SMACH State that executes a behavior tree'''
    def __init__(self, name, behavior_tree):
        smach.State.__init__(self, outcomes=['succeeded', 'failed','aborted'])
        self.name = name
        self.behavior_tree = behavior_tree

    def execute(self, userdata):
        try:
            success = self.behavior_tree.run()
            return 'succeeded' if success else 'failed'
        except Exception as e:
            rospy.loggerr(f"State {self.name} failed with error: {e}")
            return 'aborted'