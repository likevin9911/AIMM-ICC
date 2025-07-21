from constants import *
from FSM import *
from State import *
from Selectors import *
import rclpy
from rclpy.node import Node
from std_msgs import String, Bool
from geographic_msgs.msg import Twist
from sensor_msgs.msg import Image
import threading
import time

class AimmRobotNode(Node):
    '''Main ROS2 node that rybs the aimm boat'''

    def __init__(self):
        super().__init__("aimm_robot")
        
        # init mission functions
        self.missions = RosMissionFunctions(self)

        # ROS2 Publishers
        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist,'cmd_vel', 10)

        # ROS2 Subscribers
        self.buoy_sub = self.create_subscription(Bool, 'buoy_detected', self.buoy_callback, 10)
        self.obstacle_sub = self.create_subscription(Twist, 'obstacle_detected', self.obstacle_callback, 10)

        # State variables
        self.buoy_detected = False
        self.obstacle_detected = False

        # Create behavior trees
        self.setup_behavior_trees()

        # Create SMACH state machine
        self.setup_state_machine()

        # Start mission execution
        self.mission_thread = threading.Thread(target=self.run_missions)
        self.mission_thread.daemon = True
        self.mission_thread.start()

    def setup_behavior_trees(self):
        '''Create behavior trees'''

        self.calibrate_bt = Sequence([
            Action(self.missions.calibrate),
            Action(self.missions.wait)
        ])

        self.mOne_bt = Sequence([
            Action(self.missions.missionOne),
            Action(self.missions.wait)
        ])

        self.mTwo_bt = Sequence([
            Action(self.missions.missionTwo),
            Action(self.missions.wait)
            ])
        
        self.mThree_bt = Sequence([
            Action(self.missions.missionThree), 
            Action(self.missions.wait)]
            )

        self.mFour_bt = Sequence([
            Action(self.missions.missionFour), 
            Action(self.missions.wait)
            ])

        self.mFive_bt = Sequence([
            Action(self.missions.missionFive), 
            Action(self.missions.wait)
            ])

        self.mSix_bt = Sequence([Action(self.missions.missionSix), Action(self.missions.wait)])

        self.mSeven_bt = Sequence([Action(self.missions.missionSeven), Action(self.missions.wait)])

        self.mEight_bt = Sequence([Action(self.missions.missionEight), Action(self.missions.wait)])

        self.mNine_bt = Sequence([Action(self.missions.missionNine), Action(self.missions.wait)])
        
        self.failSafe_bt = Sequence([Action(self.missions.failSafe)])

    def setup_state_machines(self):
        '''Create SMACH State machine'''
        self.sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed'])
        
        with self.sm:
            # Add states with transitions
            smach.StateMachine.add("CALIBRATION", BehaviorTreesState('Calibration', self.calibrate_bt), transitions={'succeeded': 'MISSION_1', 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_1", BehaviorTreesState("Mission 1", self.mOne_bt), transitions={'succeeded': 'MISSION_2', 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_2", BehaviorTreesState("Mission 2", self.mTwo_bt), transitions={'succeeded': 'MISSION_3', 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_3", BehaviorTreesState("Mission 3", self.mThree_bt), transitions={'succeeded': "MISSION_4", 'failed': 'FAILSAFE', 'aborted': "FAILSAFE"})

            smach.StateMachine.add("MISSION_4", BehaviorTreesState("Mission 4", self.mFour_bt), transitions={'succeeded': "MISSION_5", 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_5", BehaviorTreesState("Mission 5", self.mFive_bt), transitions={'succeeded': "MISSION_6", 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_6", BehaviorTreesState("Mission 6", self.mSix_bt), transitions={'succeeded': "MISSION_7", 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_7", BehaviorTreesState("Mission 7", self.mSeven_bt), transitions={'succeeded': "MISSION_8", 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_8", BehaviorTreesState("Mission 8", self.mEight_bt), transitions={'succeeded': "MISSION_9", 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})

            smach.StateMachine.add("MISSION_9", BehaviorTreesState("Mission 9", self.mNine_bt), transitions={'succeeded': "CALIBRATION", 'failed': 'FAILSAFE', 'aborted': 'FAILSAFE'})


    def run_mission(self):
        '''Execute state machine'''
        self.get_logger().info("Startiing AIMM Mission Sequence...")

        sis = smach_ros.IntrospectionServer('aimm_smach_server', self.sm, '/AIMM_SM')
        sis.start()

        # Execute State Machine
        outcome = self.sm.execute()

        self.get_logger().info(f"Mission Sequence completed with otcome: {outcome}")
        sis.stop()

    def buoy_callback(self, msg):
        self.buoy_detected = msg.detected

        if msg.data:
            self.get_logger().info("Buoy detection signal received")

    def obstacle_callback(self, msg):
        self.obstacle_detected =  msg.data

        if msg.data:
            self.get_logger().info("Obstacle detection signal received")

    def main(args=None):
        rclpy.init(args=args)
         
        try:
            node = AimmRobotNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()

    if __name__ == '__main__':
        main()