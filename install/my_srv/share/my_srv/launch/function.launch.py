import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 启动前清理：杀死残留进程 + 清除 FastDDS 共享内存文件
    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -f web_video_server 2>/dev/null || true; '
             'pkill -9 -f "python3.*test\\.py" 2>/dev/null || true; '
             'rm -f /dev/shm/fastrtps_* 2>/dev/null || true'],
        output='screen',
        name='pre_launch_cleanup',
    )

    # 所有节点延迟 2 秒启动，确保清理完成后再绑定端口
    nodes = TimerAction(period=2.0, actions=[

        # ── 控制类节点 ──
        Node(package='my_srv', executable='glove_node.py',
             name='glove_node', output='screen',
             parameters=[{'port': '/dev/ttyAMA1'}]),
        Node(package='my_srv', executable='asr_node.py',
             name='asr_node', output='screen'),
        Node(package='my_srv', executable='face_track_node.py',
             name='face_track_node', output='screen'),

        # ── 2D 视觉节点 ──
        Node(package='my_srv', executable='color_sorting_node.py',
             name='color_sorting_node', output='screen'),
        Node(package='my_srv', executable='color_stack_node.py',
             name='color_stack_node', output='screen'),
        Node(package='my_srv', executable='color_track_node.py',
             name='color_track_node', output='screen'),
        Node(package='my_srv', executable='label_sorting_node.py',
             name='label_sorting_node', output='screen'),
        Node(package='my_srv', executable='label_stack_node.py',
             name='label_stack_node', output='screen'),
        Node(package='my_srv', executable='label_track_node.py',
             name='label_track_node', output='screen'),
        Node(package='my_srv', executable='num_sorting_node.py',
             name='num_sorting_node', output='screen'),
        Node(package='my_srv', executable='num_stack_node.py',
             name='num_stack_node', output='screen'),
        Node(package='my_srv', executable='num_track_node.py',
             name='num_track_node', output='screen'),

        # ── HTTP 服务器 ──
        Node(package='my_srv', executable='test.py',
             name='camera_http_server', output='screen'),

        # ── Aurora 930 深度相机驱动 ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.path.expanduser('~'), 'ros2_ws', 'install',
                    'deptrum-ros-driver-aurora930', 'share',
                    'deptrum-ros-driver-aurora930', 'launch',
                    'aurora930_launch.py'
                )
            ),
        ),

        # ── 深度分拣节点（启动但不活跃，等待 /depth_*/enter）──
        Node(package='my_srv', executable='depth_color_sorting_node.py',
             name='depth_color_sorting_node', output='screen'),
        Node(package='my_srv', executable='depth_label_sorting_node.py',
             name='depth_label_sorting_node', output='screen'),
        Node(package='my_srv', executable='depth_num_sorting_node.py',
             name='depth_num_sorting_node', output='screen'),

        # ── 深度码垛节点 ──
        Node(package='my_srv', executable='depth_color_stack_node.py',
             name='depth_color_stack_node', output='screen'),
        Node(package='my_srv', executable='depth_label_stack_node.py',
             name='depth_label_stack_node', output='screen'),
        Node(package='my_srv', executable='depth_num_stack_node.py',
             name='depth_num_stack_node', output='screen'),

        # ── 深度追踪节点 ──
        Node(package='my_srv', executable='depth_color_track_node.py',
             name='depth_color_track_node', output='screen'),
        Node(package='my_srv', executable='depth_label_track_node.py',
             name='depth_label_track_node', output='screen'),
        Node(package='my_srv', executable='depth_num_track_node.py',
             name='depth_num_track_node', output='screen'),

        # ── Web 视频服务器，端口9090 ──
        Node(package='web_video_server', executable='web_video_server',
             name='web_video_server', output='screen',
             parameters=[{'port': 9090}, {'max_fps': 30}, {'quality': 90}]),
    ])

    return LaunchDescription([cleanup, nodes])
