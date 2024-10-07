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

        # 속도 비례 상수
        self.alpha = 1.5  # 선속도 감소율을 결정하는 상수
        self.max_lin_vel = 0.03  # 최대 선속도
        self.max_ang_vel = 0.03  # 최대 각속도

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

        # 선속도 및 각속도 계산
        if (self.distance > self.goal_distance) and (detected_duration_seconds < self.time_threshold):
            twist = Twist()

            # 객체와의 거리에 비례한 선속도 계산 (지수 함수 사용)
            v_linear = self.max_lin_vel * (1 - math.exp(-self.alpha * (self.distance - self.goal_distance))) * max(math.cos(self.joint_state, cos(88*math.pi/180)))
            v_linear = min(self.max_lin_vel, max(0, v_linear))  # 최대 속도 제한

            # 로봇팔의 회전 각도에 따른 각속도 계산 (사인 함수 사용)
            v_angular = self.max_ang_vel * math.sin(self.joint_state)
            v_angular = max(-self.max_ang_vel, min(self.max_ang_vel, v_angular))  # 최대 각속도 제한

            twist.linear.x = v_linear  # 계산된 선속도 설정
            twist.angular.z = v_angular  # 계산된 각속도 설정

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Moving towards target: linear={v_linear:.3f}, angular={v_angular:.3f}')
        
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
