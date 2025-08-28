import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand, LaneInfo
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
SUB_LANE_TOPIC_NAME = "yolov8_lane_info"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_path_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.sub_lane_topic = self.declare_parameter('sub_lane_topic', SUB_LANE_TOPIC_NAME).value
        
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0

        # 부드러운 조향용 게인과 필터 계수
        self.steering_gain    = 0.1     # target_slope → steering 값으로 변환하는 비례 상수
        self.max_steering     = 7       # 조향의 절대 최대값 (원래 7 유지)
        self.smoothing_alpha  = 0.627     # 로우패스 필터 계수 (0~1, 1에 가까울수록 관성 커짐)

        # lane info 관련 변수 추가
        self.current_lane = "lane2"
        self.last_switch_time = 0
        self.lane_switching = False  # 현재 lane 전환 중 여부
        self.lane_switch_direction = 0  # -1: lane1→lane2, +1: lane2→lane1
        self.steering_override_until = 0  # steering 강제 만료 시간(epoch)
        
        # --- crosswalk 정지용 변수 ---
        self.crosswalk_stopping   = False   # 현재 정지 중?
        self.crosswalk_stop_until = 0.0     # 정지 해제 시각 (epoch)

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.lane_info_sub = self.create_subscription(LaneInfo, self.sub_lane_topic, self.lane_info_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        
    def lane_info_callback(self, msg: LaneInfo):
        # 전환 감지
        if msg.current_lane != self.current_lane:
            # 방향 결정
            if self.current_lane == "lane1" and msg.current_lane == "lane2":
                self.lane_switch_direction = +1   # ▶ 오른쪽 이동
            elif self.current_lane == "lane2" and msg.current_lane == "lane1":
                self.lane_switch_direction = -1   # ◀ 왼쪽 이동
            else:
                self.lane_switch_direction = 0

            # 1.5 초간 강제 조향
            self.lane_switching = True
            self.steering_override_until = time.time() + 1.5

        # 상태 갱신
        self.current_lane     = msg.current_lane
        self.last_switch_time = msg.last_switch_time


    def timer_callback(self):

        now = time.time()

        # ---------- [A] 횡단보도 정지 타이머 ----------
        if self.crosswalk_stopping and now < self.crosswalk_stop_until:
            # 4 초 정지 유지
            self.steering_command     = 0
            self.left_speed_command   = 0
            self.right_speed_command  = 0
            # lane switching 로직은 잠시 무시하도록 return
            self.get_logger().info("⏸  Crosswalk stop…")

            # 모션 명령 메시지 생성 및 퍼블리시
            motion_command_msg = MotionCommand()
            motion_command_msg.steering = self.steering_command
            motion_command_msg.left_speed = self.left_speed_command
            motion_command_msg.right_speed = self.right_speed_command
            self.publisher.publish(motion_command_msg) 

            return
        else:
            self.crosswalk_stopping = False

        # ---------- [B] Lane Switch 및 주행 타이머 ----------
        if self.lane_switching and now < self.steering_override_until:
            # lane 전환 중: steering을 강제 고정
            if self.lane_switch_direction == -1:
                self.steering_command = -7
            elif self.lane_switch_direction == 1:
                self.steering_command = 7

            self.left_speed_command = 150   # 원하는 속도로 조정
            self.right_speed_command = 150

        else:
            self.lane_switching = False

            if self.lidar_data is not None and self.lidar_data.data is True:
                # 라이다가 장애물을 감지한 경우
                self.steering_command = 0 
                self.left_speed_command = 0 
                self.right_speed_command = 0 

            elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
                # 빨간색 신호등을 감지한 경우
                for detection in self.detection_data.detections:
                    if detection.class_name=='traffic_light_red':
                        x_min = int(detection.bbox.center.position.x - detection.bbox.size.x / 2) # bbox의 좌측상단 꼭짓점 x좌표
                        x_max = int(detection.bbox.center.position.x + detection.bbox.size.x / 2) # bbox의 우측하단 꼭짓점 x좌표
                        y_min = int(detection.bbox.center.position.y - detection.bbox.size.y / 2) # bbox의 좌측상단 꼭짓점 y좌표
                        y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2) # bbox의 우측하단 꼭짓점 y좌표

                        if y_max < 150:
                            # 신호등 위치에 따른 정지명령 결정
                            self.steering_command = 0 
                            self.left_speed_command = 0 
                            self.right_speed_command = 0
            else:
                base_speed = 225
                if self.path_data is None:
                    raw_steer = 0
                else:
                    # 1) 경로의 시작과 끝 사이 기울기 계산
                    target_slope = None
                    target_slope = DMFL.calculate_slope_between_points(
                        self.path_data[-10], self.path_data[-1]
                    )
                    for det in self.detection_data.detections:
                        if det.class_name == 'road-objects':
                            base_speed = 190

                        elif det.class_name == 'crosswalk':
                            y_max = det.bbox.center.position.y + det.bbox.size.y / 2
                            if y_max > 420 and y_max < 480:           # 원하는 임계값(픽셀)으로 조정
                                self.crosswalk_stopping   = True
                                self.crosswalk_stop_until = time.time() + 8.0  # 4초 정지
                                self.get_logger().info("🚦 Crosswalk detected → 4 s stop")
                                # 정지 예약했으니 속도 0으로
                                base_speed = 0
                                # 더 볼 필요 없으므로 loop 탈출
                                break

                    # 2) 프로포셔널 제어: 기울기에 비례한 조향값
                    raw_steer = int(self.steering_gain * target_slope)
                    # 3) 최대값으로 클램프
                    raw_steer = max(-self.max_steering, min(self.max_steering, raw_steer))
                
                # 4) 로우패스 필터로 부드럽게 섞기
                self.steering_command = int(
                    self.smoothing_alpha * self.steering_command +
                    (1 - self.smoothing_alpha) * raw_steer
                )

                # (선택) 커브가 클수록 속도 낮추기
                speed_adj  = int(abs(self.steering_command) * 10.7)  # 계수는 튜닝
                speed_cmd  = max(0, base_speed - speed_adj)      # 최소 속도 0 보장
                self.left_speed_command  = speed_cmd
                self.right_speed_command = speed_cmd


        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command}")

       # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg) 

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()