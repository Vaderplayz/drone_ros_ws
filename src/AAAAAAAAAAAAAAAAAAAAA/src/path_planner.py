import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__('potential_field_planner')

        self.goal = None
        self.pose = None

        self.goal_sub = self.create_subscription(Point, "drone_goal", self.goal_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan", self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, "/mavros/setpoint_velocity/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def goal_callback(self, msg):
        self.goal = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"New goal received: {self.goal}")

    def pose_callback(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)

    def control_loop(self):
        if self.goal is None or self.pose is None:
            return

        # Attractive force towards goal
        K_att = 1.0
        force_att = K_att * (self.goal - self.pose)

        # Repulsive force from obstacles
        force_rep = np.zeros(3)
        if hasattr(self, 'ranges'):
            K_rep = 5.0
            min_dist = 2.0
            angle_min = -np.pi
            angle_inc = (2*np.pi)/len(self.ranges)
            for i, r in enumerate(self.ranges):
                if r < min_dist:
                    angle = angle_min + i * angle_inc
                    obs_vec = np.array([np.cos(angle), np.sin(angle), 0])
                    force_rep += K_rep * (1.0/r - 1.0/min_dist) * (1.0/(r**2)) * (-obs_vec)

        # Combine forces
        force = force_att + force_rep

        # Limit velocity
        max_vel = 1.0
        if np.linalg.norm(force) > max_vel:
            force = force / np.linalg.norm(force) * max_vel

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = float(force[0])
        cmd.linear.y = float(force[1])
        cmd.linear.z = float(force[2])
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
