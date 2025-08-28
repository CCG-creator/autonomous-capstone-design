import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand

#---------------Variable Setting---------------
SUB_TOPIC_NAME = "topic_control_signal"
PORT = '/dev/ttyUSB0'  # 실제 포트명에 맞게 수정
BAUD = 115200
#----------------------------------------------

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

class SerialSenderRA6M5(Node):
    def __init__(self, sub_topic=SUB_TOPIC_NAME):
        super().__init__('serial_sender_node')

        self.declare_parameter('sub_topic', sub_topic)
        self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            MotionCommand,
            self.sub_topic,
            self.data_callback,
            qos_profile
        )

    def data_callback(self, msg):
        # 값 범위 제한
        left_speed = max(0, min(999, msg.left_speed))
        right_speed = max(0, min(999, msg.right_speed))
        steering = msg.steering + 7

        # 3자리 문자열로 변환
        left_str = f"{left_speed:03d}"
        right_str = f"{right_speed:03d}"
        steer_str = f"{steering:03d}"

        # 시리얼 패킷 구성
        tx_packet = bytearray()
        tx_packet.append(0x02)
        tx_packet.extend(left_str.encode())
        tx_packet.extend(right_str.encode())
        tx_packet.extend(steer_str.encode())
        tx_packet.append(0x03)

        ser.write(tx_packet)
        self.get_logger().info(f"Sent: {list(tx_packet)}")
        # 사람이 읽을 수 있는 형태로 로그 출력
        self.get_logger().info(
            f"Sent → steering: {steer_str} , left_speed: {left_str}, right_speed: {right_str}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SerialSenderRA6M5()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutdown requested\n\n")
        # 정지 명령 전송
        stop_packet = bytearray([0x02]) + bytearray(b"000000000") + bytearray([0x03])
        ser.write(stop_packet)
    finally:
        ser.close()
        print('Serial closed')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
