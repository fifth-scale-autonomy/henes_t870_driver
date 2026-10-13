from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 2026-10-08: 노드가 실제로 읽는 파라미터만 넘긴다.
    # 예전에 넘기던 steering_offset(518), steering_range(380), max_steering(420), max_speed(250),
    # max_speed_step(15) 는 노드 코드에서 쓰이지 않아 뺐다.
    # 명령 단위 (펌웨어 firmware/henes_lcd_pid.ino 기준, 2026-10-08 실차 확인):
    #   steering_angle: 라디안, ±20도 = 제어기 ±2000
    #   speed: m/s 가 아니라 0~1 비율 -> x1000 -> 펌웨어가 0~15 단계로 줄임 (4단계도 꽤 빠름)
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/car'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5',
                              description='이 시간(s) 동안 명령이 없으면 정지'),
        DeclareLaunchArgument('wheel_diameter', default_value='0.27',
                              description='바퀴 지름 m. 주면 rpm 피드백으로 m/s 를 계산 (/vehicle/speed_mps)'),
        Node(
            package='scale_vehicle_control',
            executable='scale_vehicle_node',
            name='scale_vehicle_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                'wheel_diameter': LaunchConfiguration('wheel_diameter'),
            }],
        ),
    ])
