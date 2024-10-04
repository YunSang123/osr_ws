# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
# import numpy as np
# class TestControlNode(Node):
#     def __init__(self):
#         super().__init__('test_control')
#         self.publisher_ = self.create_publisher(Float64MultiArray, '/control/osr_controllers/drive_velocity_controller/command', 10)
#         self.timer = self.create_timer(0.2, self.publish_cmd)  # 5 Hz
#         self.end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=5)
#         # numpy 배열로 초기화 후 리스트로 변환
#         cmd_data = np.zeros(10)
#         cmd_data[:6] = 100.0
#         self.cmd = Float64MultiArray()
#         self.cmd.layout.dim.append(MultiArrayDimension(label="height", size=10, stride=10))
#         self.cmd.data = cmd_data.tolist()  # numpy 배열을 리스트로 변환하여 할당
#         # numpy 배열로 초기화 후 리스트로 변환
#         cmd_zero_data = np.zeros(10)
#         self.cmd_zero = Float64MultiArray()
#         self.cmd_zero.layout.dim.append(MultiArrayDimension(label="height", size=10, stride=10))
#         self.cmd_zero.data = cmd_zero_data.tolist()  # numpy 배열을 리스트로 변환하여 할당
#     def publish_cmd(self):
#         if self.get_clock().now() < self.end_time:
#             self.publisher_.publish(self.cmd)
#         else:
#             self.publisher_.publish(self.cmd_zero)
#             self.get_logger().debug('Done.')
#             self.destroy_timer(self.timer)  # 타이머 종료
# def main(args=None):
#     rclpy.init(args=args)
#     test_control_node = TestControlNode()
#     try:
#         rclpy.spin(test_control_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         test_control_node.destroy_node()
#         rclpy.shutdown()
# if __name__ == '__main__':
#     main()

#################

#***Before using this example the motor/controller combination must be
#***tuned and the settings saved to the Roboclaw using IonMotion.
#***The Min and Max Positions must be at least 0 and 50000
import time
from roboclaw import Roboclaw
#Windows comport name
# rc = Roboclaw("COM3",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)
def displayspeed():
	enc1 = rc.ReadEncM1(address)
	enc2 = rc.ReadEncM2(address)
	speed1 = rc.ReadSpeedM1(address)
	speed2 = rc.ReadSpeedM2(address)
	print("Encoder1:")
	# if(enc1[0]==1):
	# 	print enc1[1],
	# 	print format(enc1[2],'02x'),
	# else:
	# 	print "failed",
	# print "Encoder2:",
	# if(enc2[0]==1):
	# 	print enc2[1],
	# 	print format(enc2[2],'02x'),
	# else:
	# 	print "failed " ,
	# print "Speed1:",
	# if(speed1[0]):
	# 	print speed1[1],
	# else:
	# 	print "failed",
	# print("Speed2:"),
	# if(speed2[0]):
	# 	print speed2[1]
	# else:
	# 	print "failed "
rc.Open()
address = 0x80
version = rc.ReadVersion(address)
# if version[0]==False:
# 	print "GETVERSION Failed"
# else:
# 	print repr(version[1])
while(1):
	rc.SpeedM1(address,12000)
	rc.SpeedM2(address,-12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)
	rc.SpeedM1(address,-12000)
	rc.SpeedM2(address,12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)