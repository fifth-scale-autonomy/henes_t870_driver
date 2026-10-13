import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32, Int32
import serial

class ScaleVehicleNode(Node):
    def __init__(self):
        super().__init__('scale_vehicle_node')
        # 2026-10-08: 포트/속도/명령 대기 시간을 파라미터로 받는다 (기본값은 기존과 같음)
        self.declare_parameter('port', '/dev/car')
        self.declare_parameter('baudrate', 115200)
        # 이 시간 동안 ackermann_cmd 가 오지 않으면 정지 명령을 보낸다 (상위 노드가 죽어도 차가 달리지 않게)
        self.declare_parameter('cmd_timeout', 0.5)
        # 2026-10-08: 아두이노 펌웨어(HENES henes_lcd_pid.ino)에 맞춘 변환
        #  - 속도: 펌웨어가 Hsp 0~1000 을 받아 map(Hsp,0,1000,0,15) 로 줄인다. 예전 드라이버는 0~15 를 보내
        #          펌웨어 안에서 항상 0 이 됐다. speed(0~1) * speed_max 를 보낸다 (예전 LabVIEW 주행 기록: 약 557).
        #  - 조향: ±2000 = 조향 끝. 예전 주행 기록에서 끝이 약 ±20 도 (명령 1도당 피드백 약 98).
        #  - 주기: 펌웨어는 루프마다 새 패킷이 없으면 그 루프를 정지(Hsp=0, 중립)로 처리한다.
        #          루프가 조종기 신호 읽기로 10~30 ms 걸리므로 그보다 자주 보낸다.
        self.declare_parameter('speed_max', 1000)
        self.declare_parameter('steer_full_lock_deg', 20.0)
        self.declare_parameter('rate', 100.0)
        # 바퀴 지름 (m). 주면 rpm 피드백으로 실제 속도(m/s)를 계산해 /vehicle/speed_mps 로 낸다.
        # 펌웨어 RPM 피드백 = 6e6*E/(280*R_ms) = 실제 바퀴 rpm 의 100배 (엔코더 1회전 280카운트, 펌웨어 주석 기준)
        self.declare_parameter('wheel_diameter', 0.27)    # 실측 0.27 m (2026-10-08)
        self.declare_parameter('rpm_feedback_scale', 100.0)
        self.last_cmd_time = None
        self.timed_out = True
        self.rx_buf = bytearray()
        # 2026-10-08: 기어 전환 순서 (기본 끔). 펌웨어의 피드백 기어(Lge)는 PC 명령이 아니라 조종기 기어 스위치라
        # 켜 두면 속도가 나가지 않는다. (펌웨어 확인)
        self.declare_parameter('gear_interlock', False)
        # alive 바이트: count(기존, 매번 +1) 또는 zero(이전 아두이노 버전 드라이버처럼 항상 0).
        # 제어기가 ERP-42 처럼 14바이트(조향 뒤 브레이크) 형식이면 13바이트 패킷의 alive 자리가 브레이크로 읽혀
        # count 일 때 브레이크가 대부분 걸렸다 풀리는 가다-서기가 생길 수 있다 (2026-10-08 가설, 실차 확인 필요).
        self.declare_parameter('alive_mode', 'count')
        # 패킷 형식: 13(기존) 또는 14(ERP-42 형식: 조향 뒤에 브레이크 1바이트). brake 는 14 일 때 보낼 브레이크 값
        self.declare_parameter('packet_format', 13)
        self.declare_parameter('brake', 0)
        self.fb_gear = None
        self.shifting = False
        # 2026-10-08: 프로토콜 확인용 통계. 2초마다 로그로 남긴다 (drive 화면 값은 로그에 안 남으므로)
        self.declare_parameter('log_period', 2.0)
        self.stat = {'rx_bytes': 0, 'frames': 0, 'last_frame': None, 'last_tx': None,
                     'rpm_nonzero': 0, 'rpm_max': 0}
        self.fb_state = None          # (모드, 비상, 기어) 바뀌는 순간을 바로 기록
        self.rx_recent = bytearray()

        # Subscriber
        self.ackermann_sub = self.create_subscription(
            AckermannDrive,
            'ackermann_cmd',
            self.ackermann_callback,
            10
        )
        
        # Publishers for feedback
        self.steering_fb_pub = self.create_publisher(Float32, '/vehicle/steering_angle', 10)
        self.speed_fb_pub = self.create_publisher(Float32, '/vehicle/speed', 10)
        self.speed_mps_pub = self.create_publisher(Float32, '/vehicle/speed_mps', 10)
        self.gear_fb_pub = self.create_publisher(Int32, '/vehicle/gear', 10)
        self.mode_fb_pub = self.create_publisher(Int32, '/vehicle/mode', 10)
        # 2026-10-08: 프로토콜 확인용 원시 피드백 (조향 피드백 바이트를 해석 없이, 비상정지 바이트)
        self.steer_raw_pub = self.create_publisher(Int32, '/vehicle/steering_raw', 10)
        self.estop_fb_pub = self.create_publisher(Int32, '/vehicle/estop', 10)
        
        # 현재 상태값
        self.current_speed = 0
        self.current_steering = 0
        self.current_gear = 0
        self.mode = 1
        self.alive_counter = 0
        
        # 시리얼 연결
        try:
            self.serial = serial.Serial(
                port=self.get_parameter('port').value,
                baudrate=self.get_parameter('baudrate').value,
                timeout=0.1
            )
            self.get_logger().info('Serial port opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {str(e)}')
            raise e
        
        # 명령 전송 및 피드백 읽기 타이머 (50Hz)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.timer_callback)
        self.create_timer(self.get_parameter('log_period').value, self.log_stats)

    def ackermann_callback(self, msg):
        """Ackermann 명령 처리"""
        self.last_cmd_time = self.get_clock().now()
        if self.timed_out:
            self.get_logger().info('ackermann_cmd 수신 시작')
            self.timed_out = False
        # 속도 처리: speed 0~1 -> 펌웨어 Hsp 0~speed_max (펌웨어가 0~15 단계로 줄임)
        speed = min(abs(msg.speed), 1.0)
        if speed > 0:
            self.current_speed = int(round(speed * self.get_parameter('speed_max').value))
            self.current_gear = 0 if msg.speed > 0 else 2
        else:
            self.current_speed = 0
            self.current_gear = 1

        # 조향각 처리: rad -> ±2000 (±steer_full_lock_deg)
        full = math.radians(self.get_parameter('steer_full_lock_deg').value)
        steering = int(round(msg.steering_angle / full * 2000))
        self.current_steering = max(-2000, min(2000, steering))

    def send_command(self, speed=None):
        """LLC 프로토콜 명령 전송 (speed 를 주면 그 값으로 보냄: 기어 전환 중 0)"""
        speed = self.current_speed if speed is None else speed
        try:
            message = bytearray(b'STX')
            message.extend([
                self.mode,                           # Mode
                0,                                  # Emergency Stop
                self.current_gear,                  # Gear
                (speed >> 8) & 0xFF,                # Speed upper
                speed & 0xFF,                       # Speed lower
                (self.current_steering >> 8) & 0xFF, # Steering upper
                self.current_steering & 0xFF,        # Steering lower
            ])
            if self.get_parameter('packet_format').value == 14:
                message.append(max(0, min(200, int(self.get_parameter('brake').value))))   # Brake
            message.extend([
                (self.alive_counter if self.get_parameter('alive_mode').value == 'count' else 0),  # Alive
                0x0D,                              # CR
                0x0A                               # LF
            ])
            
            self.serial.write(message)
            self.stat['last_tx'] = bytes(message)
            self.alive_counter = (self.alive_counter + 1) % 256
            
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {str(e)}')

    def log_stats(self):
        """최근 log_period 동안의 송수신 요약을 로그로 남긴다 (프로토콜 확인용)."""
        st, per = self.stat, self.get_parameter('log_period').value
        tx = st['last_tx'].hex(' ') if st['last_tx'] else '-'
        if st['frames']:
            f = st['last_frame']
            steer_raw = (f[8] << 8) | f[9]
            self.get_logger().info(
                f'RX 패킷 {st["frames"]}개/{per:.0f}s ({st["rx_bytes"]}B) | '
                f'rpm>0 {st["rpm_nonzero"]}/{st["frames"]} 최대 {st["rpm_max"]} 평균 {st.get("rpm_sum", 0) / st["frames"]:.0f}'
                f'{self.mps_text(st.get("rpm_sum", 0) / st["frames"])} | 조향원시 {steer_raw} '
                f'(부호해석 {steer_raw - 65536 if steer_raw & 0x8000 else steer_raw}) rpm {(f[6] << 8) | f[7]} '
                f'모드 {f[3]} 비상 {f[4]} 기어 {f[5]} | RX {f.hex(" ")} | TX {tx}')
        elif st['rx_bytes']:
            self.get_logger().warn(
                f'RX {st["rx_bytes"]}B 받았지만 STX..CRLF 13바이트 패킷 없음 | 최근 원본 {bytes(self.rx_recent).hex(" ")} '
                f'| TX {tx}')
        else:
            self.get_logger().warn(f'제어기에서 받은 데이터 없음 | TX {tx}')
        st['rx_bytes'] = st['frames'] = st['rpm_nonzero'] = st['rpm_max'] = st['rpm_sum'] = 0

    def mps_text(self, rpm_fb):
        d = self.get_parameter('wheel_diameter').value
        if d <= 0:
            return ''
        return f' (약 {rpm_fb / self.get_parameter("rpm_feedback_scale").value * math.pi * d / 60:.2f} m/s)'

    def stop_values(self):
        """속도 0, 중립 기어, 조향 0"""
        self.current_speed = 0
        self.current_gear = 1
        self.current_steering = 0

    def timer_callback(self):
        """명령 전송 및 피드백 처리"""
        age = (None if self.last_cmd_time is None
               else (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9)
        if age is None or age > self.get_parameter('cmd_timeout').value:
            if not self.timed_out:
                self.get_logger().warn(f'ackermann_cmd 가 {age:.2f} s 동안 없음 -> 정지')
                self.timed_out = True
            self.stop_values()
        speed = self.current_speed
        if (self.get_parameter('gear_interlock').value and speed > 0 and self.current_gear in (0, 2)
                and self.fb_gear is not None and self.fb_gear != self.current_gear):
            speed = 0
            if not self.shifting:
                self.get_logger().info(f'기어 전환 대기: 피드백 {self.fb_gear} -> 요청 {self.current_gear} (속도 0 유지)')
                self.shifting = True
        elif self.shifting:
            self.get_logger().info(f'기어 전환 완료: {self.fb_gear}')
            self.shifting = False
        self.send_command(speed)
        
        # 피드백 데이터 읽기
        # 2026-10-08: 13바이트씩 그대로 읽으면 한 번 어긋난 뒤로는 계속 어긋나 피드백이 끊긴다.
        # 받은 바이트를 모아 'STX'로 시작하고 CR LF 로 끝나는 13바이트 패킷을 찾아 쓴다 (해석은 기존과 같음).
        try:
            if self.serial.in_waiting:
                chunk = self.serial.read(self.serial.in_waiting)
                self.rx_buf += chunk
                self.stat['rx_bytes'] += len(chunk)
                self.rx_recent = (self.rx_recent + chunk)[-64:]
            data = None
            while True:
                i = self.rx_buf.find(b'STX')
                if i < 0:
                    self.rx_buf = self.rx_buf[-2:]          # 'ST' 가 잘려 있을 수 있어 2바이트는 남긴다
                    break
                if len(self.rx_buf) - i < 13:
                    self.rx_buf = self.rx_buf[i:]
                    break
                frame = bytes(self.rx_buf[i:i + 13])
                if frame[11] == 0x0D and frame[12] == 0x0A:
                    data = frame                            # 가장 최근의 완전한 패킷
                    self.rx_buf = self.rx_buf[i + 13:]
                else:
                    self.rx_buf = self.rx_buf[i + 1:]       # 우연히 'STX' 처럼 보인 바이트는 건너뜀
            if len(self.rx_buf) > 4096:
                self.rx_buf = self.rx_buf[-13:]

            if data is not None:
                self.stat['frames'] += 1
                self.stat['last_frame'] = data
                rpm_now = (data[6] << 8) | data[7]
                rpm_now = rpm_now - 0x10000 if rpm_now & 0x8000 else rpm_now   # 후진은 음수
                if rpm_now:
                    self.stat['rpm_nonzero'] += 1
                    self.stat['rpm_max'] = max(self.stat['rpm_max'], abs(rpm_now))
                self.stat['rpm_sum'] = self.stat.get('rpm_sum', 0) + rpm_now
                state = (data[3], data[4], data[5])
                if state != self.fb_state:
                    self.get_logger().info(
                        f'피드백 변화: 모드 {state[0]} 비상 {state[1]} 기어 {state[2]} '
                        f'(이전 {self.fb_state}) | 보낸 명령 기어 {self.current_gear} 속도 {self.current_speed}')
                    self.fb_state = state
                # 모드 피드백
                mode_msg = Int32()
                mode_msg.data = data[3]
                self.mode_fb_pub.publish(mode_msg)

                # 기어 피드백
                gear_msg = Int32()
                gear_msg.data = data[5]
                self.fb_gear = data[5]
                self.gear_fb_pub.publish(gear_msg)

                # RPM 피드백
                rpm = (data[6] << 8) | data[7]
                rpm = rpm - 0x10000 if rpm & 0x8000 else rpm   # 부호 있는 16비트 (후진 음수)
                speed_msg = Float32()
                speed_msg.data = float(rpm)
                self.speed_fb_pub.publish(speed_msg)
                d = self.get_parameter('wheel_diameter').value
                if d > 0:
                    wheel_rpm = rpm / self.get_parameter('rpm_feedback_scale').value
                    self.speed_mps_pub.publish(Float32(data=float(wheel_rpm * math.pi * d / 60.0)))

                self.estop_fb_pub.publish(Int32(data=data[4]))
                self.steer_raw_pub.publish(Int32(data=(data[8] << 8) | data[9]))

                # 조향각 피드백 (-2000~2000 범위에서 라디안으로 변환)
                steer = (data[8] << 8) | data[9]
                steer = steer - 0x10000 if steer & 0x8000 else steer      # 부호 있는 16비트
                steering_msg = Float32()
                steering_msg.data = float(steer) / 2000.0 * math.radians(
                    self.get_parameter('steer_full_lock_deg').value)   # 라디안으로 변환
                self.steering_fb_pub.publish(steering_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to read feedback: {str(e)}')
            self.serial.reset_input_buffer()
            self.rx_buf = bytearray()

    def __del__(self):
        """종료 처리"""
        if hasattr(self, 'serial') and self.serial.is_open:
            final_message = bytearray(b'STX')
            final_message.extend([1, 0, 1, 0, 0, 0, 0, 0, 0x0D, 0x0A])
            self.serial.write(final_message)
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScaleVehicleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # 종료 시 정지 명령을 확실히 보낸다 (__del__ 은 호출이 보장되지 않음)
        if node is not None and hasattr(node, 'serial') and node.serial.is_open:
            node.stop_values()
            for _ in range(3):
                node.send_command()
            node.serial.flush()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
