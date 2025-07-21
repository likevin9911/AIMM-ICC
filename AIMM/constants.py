import rclpy
from rclpy.node import Node
from std_msgs import String, Bool
from geographic_msgs.msg import Twist
from sensor_msgs.msg import Image
import threading
import time



class RosMissionFunctions:
    def __init__(self, node):
        self.node = node

    def calibrate(self):
        time_count = 0
        self.node.get_logger().info("Making sure systems are good...")

        while time_count <= 2:
            time_count += 1
            time.sleep(1)

            if time_count == 3:
                self.node.get_logger().info("Systems Ready!")
                self.publish_status("Systems Ready")
                return True
            else:
                self.node.get_logger().error("Systems NOT Ready")
                return False
            
    def wait(self):
        self.node.get_logger().info("Awaiting further instruction...")
        self.publish_status("Waiting")
        time.sleep(1)
        return True

    def traverse(self):
        self.node.get_logger().info("Navigating...")


        # Publish movement command to ROS
        cmd = Twist()
        cmd.linear.x = 0.5 # Move forward
        self.node.cmd_vel_pub.publish(cmd)
        time.sleep(2)

        # Stop
        cmd.linear.x = 0.0
        self.node.cmd_vel_pub.publish(cmd)

        return True
    
    def buoyDetected(self):
        if self.node.buoy_detected:
            self.node.get_logger().info("RED and GREEN Buoy detected!")
            self.node.get_logger().info("Navigating in between buoys...")
            self.publish_status("Navigating between buoys")
            # Logic for navigation in between buoys here
            return True
        return False

    def failSafe(self):
        self.node.get_logger().warn("FAILSAFE ACTIVATED - $PINNY $PIN")
        cmd = Twist()
        cmd.angular.z = 1.0 # Spin
        self.node.cmd_vel_pub.publish(cmd)
        time.sleep(3)
        cmd.angular.z = 0.0
        self.node.cmd_vel_pub.publish(cmd)
        return True
    
    def doge(self):
        self.node.get_logger().info("Object within 5ft...")
        self.traverse()
        self.node.get_logger().info("Fleeted Successfully!")
        return True
    
    # ======================= MISSIONS ==================================

    def missionOne(self):
        '''Navigate between buoys'''

        self.traverse()
        
        time.sleep(2)

        if self.buoyDetected():
            return True
        return False
    
    def missionTwo(self):
        '''Object avoidance'''
        self.traverse()

        if self.node.obstacle_detected:
            self.node.get_logger().info("Object within 5ft...")
            self.traverse()
            self.node.get_logger().info("Fleeted Successfully!")
        return True
    
    def missionThree(self):
        '''Enemy Evasion'''
        self.traverse()
        self.node.get_logger().info("Enemies Detected VIA RADAR...")
        self.traverse()
        self.node.get_logger().info("Evade Complete!")
        return True
    
    def missionFour(self):
        '''Target interaction'''
        self.traverse()
        self.node.get_logger().info("Detected Target")
        self.node.get_logger().info("Booping Target...")

        # Simulate target interaction
        time.sleep(2)
        self.node.get_logger().info("Booping Completed!")

        self.traverse()
        return True
    
    def missionFive(self):
        '''Deploy collection center'''
        self.traverse()
        self.node.get_logger().info("Deploying...")

        # Simulate deployment
        time.sleep(3)
        self.node.get_logger().info("Collection Center dropped...")
        return True
    
    def missionSix(self):
        '''Drone Op'''
        self.traverse()
        self.wait()

        self.node.get_logger().info("Communicating to DRONE...")
        self.node.get_logger().info("DRONE launching...")
        time.sleep(5) # simulate drone
        self.node.get_logger().info("DRONE delivered package...")
        self.node.get_logger().info("DRONE returning...")
        time.sleep(3)
        self.node.get_logger().info("DRONE Docked.")

        self.traverse()
        return True
    
    def missionSeven(self):
        '''Cargo retrieval'''
        self.traverse()

        self.node.get_logger().info("RADAR looking for cargo...")
        self.node.get_logger().info("Zed 2i looking for cargo...")
        time.sleep(3) # simulate search
        self.node.get_logger().info("CARGO found...")

        self.wait()
        self.node.get_logger().info("Scooping CARGO...")
        time.sleep(2)
        self.node.get_logger().info("CARGO retrieved.")
        return True
    
    def missionEight(self):
        '''RASPI Coms'''
        self.traverse()
        self.wait()

        self.node.get_logger().info("Communicating with RASPI...")
        self.node.get_logger().info("RASPI in DM State...")
        time.sleep(2)

        self.traverse()
        return True

    def missionNine(self):
        '''Return Home'''
        self.node.get_logger().info("Returning home...")
        self.traverse()
        self.node.get_logger().info("Arrived.")
        return True

    def publish_status(self, status):
        '''Publish mission stats to ROS2 topic'''
        msg = String()
        msg.data = status
        self.node.status_pub.publish(msg)