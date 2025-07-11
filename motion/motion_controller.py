import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, pow, atan2

class MyNode(Node):
	def __init__(self):
		super().__init__('turtlebot_controller')
		self.declare_parameter('goal_x', 5.0)
		self.declare_parameter('goal_y', 5.0)
		self.declare_parameter('tolerance', 0.1)
		self.pose = Pose()
		self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose',self.callback, 10)
		timer_period = 1
		self.timer = self.create_timer(timer_period, self.move_turtle)
	def move_turtle(self):
		goal = Pose()
		goal.x = self.get_parameter('goal_x').get_parameter_value().double_value
		goal.y = self.get_parameter('goal_y').get_parameter_value().double_value
		tolerance = self.get_parameter('tolerance').get_parameter_value().double_value

		distance = self.euclidean_distance(goal)
		twist = Twist()
		if self.pose.x == 0.0 and self.pose.y == 0.0 and self.pose.theta == 0.0:
			self.get_logger().warn("Waiting for initial pose...")
			return
		if distance > tolerance:
			twist.linear.x = self.linear_vel(goal)
			twist.linear.y = 0.0
			twist.linear.z = 0.0
			twist.angular.x =0.0
			twist.angular.y = 0.0
			twist.angular.z = self.angular_vel(goal)
		else :
			twist.linear.x = 0.0
			twist.angular.z = 0.0

		self.velocity_publisher.publish(twist)
		self.get_logger().info(f"Pose: ({self.pose.x:.2f}, {self.pose.y:.2f}) | Goal: ({goal.x:.2f}, {goal.y:.2f}) | Distance: {distance:.2f}")
	def callback(self, msg):
		self.pose = msg
		self.get_logger().info(f"x: {msg.x}, y: {msg.y}, theta: {msg.theta}")

	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.x - self.pose.x), 2) +pow((goal_pose.y - self.pose.y), 2))

	def linear_vel(self, goal_pose, constant=1.5):
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

	def angular_vel(self, goal_pose, constant=6):
		return constant * (self.steering_angle(goal_pose) - self.pose.theta)

def main():
	rclpy.init()
	node = MyNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
