import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import sys
sys.path.append("/home/reike/Desktop/scrivania/robotics/simulations&notebooks/infinity-autopilot-simulation/uav_core")

from simulator import step

class SimNode(Node):
    def __init__(self):
        super().__init__('uav_sim')
        self.sub = self.create_subscription(
            Wrench, '/cmd_wrench', self.cmd_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.thrust = 0.0
        self.timer = self.create_timer(0.01, self.timer_callback)

    def cmd_callback(self, msg):
        self.thrust = msg.force.z

    def timer_callback(self):
        state = step(self.thrust)
        odom = Odometry()
        odom.pose.pose.position.x = state.pos[0]
        odom.pose.pose.position.y = state.pos[1]
        odom.pose.pose.position.z = state.pos[2]
        self.pub.publish(odom)

def main():
    rclpy.init()
    node = SimNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
