import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool
import math
from sensor_msgs.msg import JointState

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.is_goal_pub = self.create_publisher(Bool, 'rover/is_goal', 10)

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
        self.goal_distance = 0.4
        self.x = 0
        self.y = 0
        self.z = 0
        self.distance = 0.5

        self.clock = Clock()
        self.last_detected = self.clock.now()
        self.time_threshold = 3.0

        # 속도 비례 상수
        self.alpha = 1.5  # 선속도 감소율을 결정하는 상수
        self.max_lin_vel = 0.02  # 최대 선속도
        self.max_ang_vel = 0.03  # 최대 각속도

        # 상태 변수 추가
        self.goal_reached = False
        self.stop_timer = None

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
        print("self.distance : {}".format(self.distance))
        print("detected_duration_seconds : {}".format(detected_duration_seconds))

        # 선속도 및 각속도 계산
        if (self.distance > self.goal_distance) and (detected_duration_seconds < self.time_threshold):
            twist = Twist()

            # 객체와의 거리에 비례한 선속도 계산 (지수 함수 사용)
            v_linear = self.max_lin_vel * (1 - math.exp(-self.alpha * (self.distance - self.goal_distance))) * max(math.cos(self.joint_state), math.cos(88 * math.pi / 180))
            v_linear = min(self.max_lin_vel, max(0, v_linear))  # 최대 속도 제한

            # 로봇팔의 회전 각도에 따른 각속도 계산 (사인 함수 사용)
            v_angular = self.max_ang_vel * math.sin(self.joint_state)
            v_angular = max(-self.max_ang_vel, min(self.max_ang_vel, v_angular))  # 최대 각속도 제한

            twist.linear.x = v_linear  # 계산된 선속도 설정
            twist.angular.z = v_angular  # 계산된 각속도 설정

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'Moving towards target: linear={v_linear:.3f}, angular={v_angular:.3f}')

        # 객체와 일정 거리를 유지함.
        elif (self.distance <= self.goal_distance and not self.goal_reached):
            self.goal_reached = True  # 목표에 도달한 상태로 설정
            self.start_stop_timer()

    def start_stop_timer(self):
        self.stop_timer = self.create_timer(0.5, self.stop_robot)  # 0.5초 후 stop_robot 호출

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.01
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('목표 거리 도달, 정지 중입니다')

        goal = Bool()
        goal.data = True
        self.is_goal_pub.publish(goal)

        # 타이머 제거
        self.stop_timer.cancel()
        self.stop_timer = None

        # 노드 종료
        self.get_logger().info('Stopping robot and shutting down...')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    print("break and destroy node")

if __name__ == '__main__':
    main()
