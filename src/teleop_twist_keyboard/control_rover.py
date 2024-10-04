import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
import math
from sensor_msgs.msg import JointState

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.pose_subscriber = self.create_subscription(
            PointStamped,
            'object_detector/position',
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1초 주기로 타이머 생성

        self.joint_state = 0
        self.joint_threshold = math.radians(5)
        self.goal_distance = 0.38
        self.x = 0
        self.y = 0
        self.z = 0
        self.distance = 0

        self.clock = Clock()
        self.last_detected = self.clock.now()
        self.time_threshold = 3.0
        self.max_lin_vel = 0.1

    def joint_callback(self, msg):
        self.joint_state = msg.position[0]

    def pose_callback(self, msg):
        self.last_detected = self.clock.now()
        self.x = msg.point.x
        self.y = msg.point.y
        self.z = msg.point.z
        self.distance = math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def timer_callback(self):
        self.drive_robot()  # 주기적으로 로봇 주행 함수 호출

    def drive_robot(self):
        detected_duration = self.clock.now() - self.last_detected
        detected_duration_seconds = detected_duration.nanoseconds / 1e9  # 나노초를 초로 변환

        if (self.distance > self.goal_distance) and (detected_duration_seconds < self.time_threshold):
            twist = Twist()
            if self.joint_state > self.joint_threshold:
                twist.linear.x = 0.01
                twist.angular.z = 0.01
                print("좌회전")
            elif self.joint_state < -self.joint_threshold:
                twist.linear.x = 0.01
                twist.angular.z = -0.01
                print("우회전")
            else:
                twist.linear.x = 0.01
                twist.angular.z = 0.00
                print("직진")

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('목표를 향해 이동 중입니다')
        
        # 객체와 일정 거리를 유지함.
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('목표 거리 도달, 정지 중입니다')

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
