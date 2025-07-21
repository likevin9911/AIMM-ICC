import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('Motor__Controller')

        self.motor_pwm_pin_left = 18
        self.motor_pwm_pin_right = 19

        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.motor_pwm_pin_left, GPIO.OUT)
        GPIO.setup(self.motor_pwm_pin_right, GPIO.OUT)


        motor_pwm_pin_left = GPIO.PWM(self.motor_pwm_pin_left, 1000)
        motor_pwm_pin_right = GPIO.PWM(self.motor_pwm_pin_right, 1000)


        self.motor_pwm_pin_left.start(0)
        self.motor_pwm_pin_right.start(0)

        self.create_subsciption(Twist, 'cmd_vel', self.control_motors, 10)

    def control_motors(self, msg):
        left_speed = msg.linear.x - msg.angular.z
        right_speed = msg.linear.x + msg.angular.z

        left_speed = max(0, min(100, left_speed = 100))
        right_speed = max(0, min(100, right_speed = 100))

        self.motor_left_pwm.ChangeDutyCycle(left_speed)
        self.motor_right_pwm.ChangeDutyCycle(right_speed)

        self.get_logger().info(f"Left Motor Speed: {left_speed}% | Right Moror Speed: {right_speed}%")

    def destroy_node(self):
        self.motor_pwm_pin_left.stop()
        self.motor_pwm_pin_right.stop()

        GPIO.cleanup()

        super().destroy_node()

    def main(args=None):
        rclpy.init(args=args)
        node = MotorControllerNode()

        rclpy.spin(Node)
        rclpy.shutdown()
    
    if __name__ == "__main__":
        main()
