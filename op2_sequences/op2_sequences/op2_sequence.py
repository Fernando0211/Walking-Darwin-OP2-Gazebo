import rclpy
from rclpy.action import ActionClient
from op2_msgs.action import Op2Task

class NumberPublisher:
    def __init__(self):
        self.node = rclpy.create_node('number_publisher')
        self.action_client = ActionClient(self.node, Op2Task, 'op2_rutines')
        self.goal_number = 0 
        self.send_goal()

    def send_goal(self):
        goal_msg = Op2Task.Goal()
        goal_msg.task_number = self.goal_number
        self.node.get_logger().info('Sending goal: {}'.format(self.goal_number))
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info('Received feedback: {0}%'.format(feedback_msg.percentage))

    def result_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_received_callback)
        else:
            self.node.get_logger().info('Goal was rejected.')

    def result_received_callback(self, future):
        result = future.result()
        if result:
            if result.result.success:
                self.node.get_logger().info('Goal achieved successfully!')
                self.goal_number += 1
                if self.goal_number <= 7:
                    self.send_goal()
                else:
                    self.node.get_logger().info('Reached the limit of tasks.')
            else:
                self.node.get_logger().info('Goal failed.')
        else:
            self.node.get_logger().info('Goal was cancelled.')

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node.node)
    node.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
