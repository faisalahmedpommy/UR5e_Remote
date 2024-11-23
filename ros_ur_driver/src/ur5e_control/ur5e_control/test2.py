import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointState, '/ur5e/joint_command', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # publish every 0.1 seconds
        self.joint_state = JointState()

        # Replace these with your actual joint names
        self.joint_state.name = [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint', 
            'wrist_1_joint', 
            'wrist_2_joint', 
            'wrist_3_joint'
        ]

        # Initialize joint positions, velocities, and efforts if necessary
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def timer_callback(self):
        # Example: Increment each joint position in a simple pattern
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Example movement pattern for each joint
        self.joint_state.position[0] -= 0.01  # Increment shoulder_pan_joint
        self.joint_state.position[1] -= 0.01  # Increment shoulder_lift_joint
        self.joint_state.position[2] += 0.01  # Increment elbow_joint
        self.joint_state.position[3] += 0.01  # Increment wrist_1_joint
        self.joint_state.position[4] += 0.01  # Increment wrist_2_joint
        self.joint_state.position[5] += 0.01  # Increment wrist_3_joint

        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f'Publishing joint states: {self.joint_state.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()

    try:
        rclpy.spin(joint_command_publisher)
    except KeyboardInterrupt:
        pass

    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
