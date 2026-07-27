import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    ws_root = os.path.dirname(current_dir)

    # 1. Livox 驱动
    livox_launch_dir = os.path.join(ws_root, 'src', 'livox_ros_driver2', 'launch_ROS2')
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_launch_dir, 'msg_MID360_launch.py')
        )
    )

    # 2. FAST-LIO 
    fast_lio_launch_dir = os.path.join(ws_root, 'src', 'FAST_LIO_ROS2', 'launch')
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_launch_dir, 'mapping.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # launch_LivoxToPointCloud2_node = Node(
    #     package='pointcl',
    #     executable='livox_to_pcl',
    #     name='livox_to_pointcloud2',
    #     output='screen'
    # )

    # 3. Odom to Pose 节点
    launch_odom_to_pose_node = Node(
        package='offboard',
        executable='odom_to_pose_node',
        name='odom_to_pose_node',
        output='screen'
    )

    # 4. MAVROS
    mavros_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch',
                'px4.launch'#nimad，这是ros1的不知哦道哪个沙比ai
            )
        ),
        launch_arguments={
            'fcu_url': '/dev/ttyACM0:921600',
            'gcs_url': 'udp://@192.168.43.6'
        }.items()
    )

    # mavros_launch = IncludeLaunchDescription(
    #     FrontendLaunchDescriptionSource(  # ← 用 Frontend，不是 Python
    #         os.path.join(
    #             get_package_share_directory('mavros'),
    #             'launch',
    #             'px4.launch'
    #         )
    #     ),
    #     launch_arguments={
    #         'fcu_url': '/dev/ttyACM0:921600',
    #         'gcs_url': 'udp://@192.168.43.6'
    #     }.items()
    # )

    # 5. 任务控制节点
    launch_task_controller_node = Node(
        package='offboard',
        executable='offb_node',
        name='offb_node',
        output='screen'
    )

    # 使用 TimerAction 串联启动，保持原有延时效果
    return LaunchDescription([
        # 立即启动 Livox
        livox_launch,

        # 3 秒后启动 FAST-LIO 
        TimerAction(
            period=3.0,
            actions=[fast_lio_launch]
        ),

        # 6 秒后启动 OdomToPose
        TimerAction(
            period=6.0,
            actions=[launch_odom_to_pose_node]
        ),

        # 9 秒后启动 MAVROS
        TimerAction(
            period=9.0,
            actions=[mavros_launch]
        ),

        # 12 秒后启动任务控制器
        TimerAction(
            period=12.0,
            actions=[launch_task_controller_node]
        ),
    ])


# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription,TimerAction,DeclareLaunchArgument,RegisterEventHandler
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from launch.event_handlers import OnProcessStart,OnIncludeLaunchDescription
# from ament_index_python.packages import get_package_share_directory


# # ros2 launch 就依附于这样一个文件
# def generate_launch_description():
#     # livox_ros_driver2_node=Node(

#     # )
#     # 我们下面这个是应用的官方启动文件，所以直接用include方法
#     current_dir=os.path.dirname(__file__)#this method will gain his father path,所以，——currentdir是launch不是launch/****.launch.py
#     ws_root = os.path.dirname(current_dir)#我们这样不会写死路径，这是根目录。
#     # livox driver启动
#     livox_launch_dir=os.path.join(ws_root,'src','livox_ros_driver2','launch_ROS2')#这个是把路径组合起来的
#     livox_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 livox_launch_dir,'msg_MID360_launch.py'
#             )
#         )
#     )


#     # fast-lio与pointcloud转换节点启动
#         # 1.fastlio
#     fast_lio_launch_dir = os.path.join(ws_root,'src','FAST_LIO_ROS2','launch')
#     fast_lio_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(fast_lio_launch_dir,'mapping.launch.py')
            
#         ),
#         launch_arguments={'rviz':'false'}.items()#
#     )
#         # 2.livox custommsg to pointcloud2
#     launch_LivoxToPointCloud2_node = Node(
#         package='pointcl',
#         executable = 'livox_to_pcl',#这边写可执行文件的名字
#         name= 'livox_to_pointcloud2',
#         output = "screen"
#     )
#     delayed_fast_lio = TimerAction(#这是个三秒定时器，三秒后会执行，由人ros2官方提供
#         period=3.0,
#         actions=[fast_lio_launch,launch_LivoxToPointCloud2_node]
#     )

#     #启动offboard中的odom to pose中的“odom_to_pose_node”
#     launch_odom_to_pose_node = Node(
#         package='offboard',
#         executable='odom_to_pose_node',
#         name='odom_to_pose_node',
#         output = "screen"#和一位
#     )
#     delayed_odom_to_pose_node = TimerAction(
#         period=6.0,
#         actions = [launch_odom_to_pose_node]
#     )
#     event_handler_otp_node = RegisterEventHandler(
#         OnIncludeLaunchDescription(
#             matcher=fast_lio_launch,#target指的是监听对象
#             on_include=[delayed_odom_to_pose_node]#on_start里面可以写好多的动作，包括结点运行，包含启动文件等等
#         )
#     )

#     #mavros启动
#     mavros_launch=IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('mavros'),
#                 'launch',
#                 'px4.launch'
#             )
#         ),
#         launch_arguments={
#             'fcu_url':'/dev/ttyACM0:921600',
#             'gcs_url' : 'udp://@192.168.43.6'
#         }.items()
#     )
#     event_handler_mavros = RegisterEventHandler(
#         OnProcessStart(
#             target_action=launch_odom_to_pose_node,
#             on_start=[mavros_launch]
#         )
#     )
#     #任务控制节点启动
#     launch_task_controller_node = Node(
#         package='offboard',
#         executable='offb_node',
#         name='offb_node',
#         output = "screen"#和一位
#     )
#     delayed_task_controller= TimerAction(
#         period=3.0,
#         actions = [launch_task_controller_node]
#     )
#     event_handler_tc_node = RegisterEventHandler(
#         OnIncludeLaunchDescription(
#             matcher=mavros_launch,
#             on_include=[delayed_task_controller]
#         )
#     )

#     return LaunchDescription([
#         livox_launch,
#         delayed_fast_lio,
#         event_handler_otp_node,
#         event_handler_mavros,
#         event_handler_tc_node,
#     ])

# # 总结：ai说我这个不太完美，因为我只是固定延迟了，没有去监听他是否真的运行起来了
