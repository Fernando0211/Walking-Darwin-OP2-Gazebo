import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/robot_controller/joint_trajectory',
            10)

    def listener_callback(self, msg):
        joint_trajectory_msg = JointTrajectory()
        
        joint_trajectory_msg.joint_names = msg.name
        
        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.velocities = msg.velocity
        point.accelerations = [0.0] * len(msg.position)
        point.effort = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0
        
        joint_trajectory_msg.points.append(point)
        
        self.publisher.publish(joint_trajectory_msg)
        self.get_logger().info('Publicando JointTrajectory')

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
