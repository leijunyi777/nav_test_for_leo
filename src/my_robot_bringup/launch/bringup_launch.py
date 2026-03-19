import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'my_robot_bringup'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 路径定义
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'default.rviz')
    nav2_real_params_file = os.path.join(pkg_share, 'config', 'nav2_real_params.yaml')

    #rplidar_launch_path = '/home/student40/Robotic_systems_design/nav2/src/rplidar_ros/launch/rplidar_a2m12_launch.py'
    rplidar_dir = get_package_share_directory('rplidar_ros')
    rplidar_launch_path = os.path.join(rplidar_dir, 'launch', 'rplidar_a2m12_launch.py')
    
    # 1. Joy Node (读取手柄硬件)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0', # 请确保手柄是 js0，否则改为 js1
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # 2. Teleop Twist Joy (手柄 -> cmd_vel)
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'axis_linear.x': 1,        # 左摇杆上下
            'axis_angular.yaw': 0,     # 左摇杆左右 (如果是右摇杆通常是 3)
            'scale_linear.x': 0.8,     # 加快一点速度以便测试
            'scale_angular.yaw': 1.0,
            'enable_button': -1,       # -1 或 0 配合 require_enable_button: false 实现无按键直接控制
            'require_enable_button': False
        }]
    )


    # 3. 启动雷达 (引用官方 launch)
    # 假设你的雷达挂载在 /dev/ttyUSB0，且需要启动 rplidar_a2m12.launch.py
    # 确保你已经安装 sudo apt install ros-jazzy-rplidar-ros
    if not os.path.exists(rplidar_launch_path):
        print(f"Warning: Rplidar launch file not found at: {rplidar_launch_path}")

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',  # 确保端口正确
            'frame_id': 'laser',
            'range_min': '0.4'             # 确保 frame_id 匹配 TF
        }.items()
    )
    # 4. 发布 TF (base_link -> laser)
    # ---------------------------------------------------------
    # 参数顺序: x y z yaw pitch roll parent_frame child_frame
    # 修改点：
    # --x 0.035   (前方 35mm)
    # --z -0.137  (下方 137mm)
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = [
            '--x', '0.02313', 
            '--y', '0.0', 
            '--z', '-0.138', 
            '--yaw', '3.14159', 
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'base_link', 
            '--child-frame-id', 'laser'
        ]
    )

    # 5. 启动 Slam Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params_file, 'use_sim_time': 'false'}.items()
    )

    # ================= 启动 Nav2 (纯导航模式) =================
    nav2_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',  # <== 真实硬件必须为 false
            'params_file': nav2_real_params_file
        }.items()
    )

    # 7. 启动 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    # 8. 启动四轮过滤器
    four_wheel_filter_node = Node(
        package='my_robot_bringup',
        executable='four_wheel_filter',
        name='four_wheel_filter',
        output='screen',
        parameters=[{
        'use_sim_time': False, # 注意：在 Python 字典里可以直接用布尔值 False
        }]
    )

    # =========================================================
    # 9. 启动里程计重置触发节点 (只执行一次)
    # =========================================================
    reset_odom_trigger_node = Node(
        package='my_robot_bringup',
        executable='reset_odometry',  # 对应 setup.py 里的注册名
        name='reset_odometry_trigger',
        output='screen'
    )

    return LaunchDescription([
        #reset_odom_trigger_node,
        rplidar_launch,
        tf_base_to_laser,
        slam_launch,
        nav2_bringup_node,
        #joy_node,
        #teleop_twist_joy,
        rviz_node,
        four_wheel_filter_node,
    ])