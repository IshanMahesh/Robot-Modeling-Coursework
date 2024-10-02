import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('tb_openLoop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Parameters for scnario 1
        self.v = 0.1  # Cnst velocity (m/s)
        self.distance = 1.0  # Dist (m)
        #Time to reach
        self.time_to_move = self.distance / self.v
        
        # Scenario 2 parameters
        self.acceleration = 0.05  # in m/s^2
        self.final_speed = 0.2  # in m/s
        self.x2 = 1.0  # total dist
 
    def timer_callback(self):
        # Scenario 1
        self.run_constant_velocity()

        # Scenario 2
        #self.run_acceleration_profile()

    def run_constant_velocity(self):
        start_time = time.time()
        while time.time() - start_time < self.time_to_move:
            msg = Twist()
            msg.linear.x = self.v
            self.publisher_.publish(msg)
            time.sleep(0.1)
        # Stop the robot
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def run_acceleration_profile(self):
        # 1: Accelerate -> final speed
        speed = 0.0
        while speed < self.final_speed:
            speed += self.acceleration * 0.1  # Update speed
            msg = Twist()
            msg.linear.x = speed
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # 2: Move -> constant speed
        move_time = self.x2 / self.final_speed
        start_time = time.time()
        while time.time() - start_time < move_time:
            msg = Twist()
            msg.linear.x = self.final_speed
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # 3: Decelerating to stop
        while speed > 0:
            speed -= self.acceleration * 0.1
            msg = Twist()
            msg.linear.x = max(speed, 0.0)
            self.publisher_.publish(msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    open_loop_controller = OpenLoopController()
    rclpy.spin(open_loop_controller)

    open_loop_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
