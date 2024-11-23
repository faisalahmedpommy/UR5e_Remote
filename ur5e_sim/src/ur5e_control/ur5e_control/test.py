import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR5eController(Node):

    def __init__(self):
        super().__init__('ur5e_controller')

        # Create a publisher for the joint trajectory controller
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_joints_controller/joint_trajectory',
            10
        )

        # Define the joint names (make sure they match the names in your URDF)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Send an initial trajectory
        self.send_trajectory()

    def send_trajectory(self):
        # Create a JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]  # example positions (in radians)
        point.time_from_start.sec = 5  # move to the position in 5 seconds

        # Add the point to the trajectory
        trajectory_msg.points.append(point)

        # Publish the trajectory
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Trajectory sent!')

def main(args=None):
    rclpy.init(args=args)
    ur5e_controller = UR5eController()

    # Keep the node alive to listen for any callbacks
    rclpy.spin(ur5e_controller)

    # Shutdown
    ur5e_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
