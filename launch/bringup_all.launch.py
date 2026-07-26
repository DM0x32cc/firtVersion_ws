import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction,DeclareLaunchArgument,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart,OnIncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


# ros2 launch 就依附于这样一个文件
def generate_launch_description():
    # livox_ros_driver2_node=Node(

    # )
    # 我们下面这个是应用的官方启动文件，所以直接用include方法
    current_dir=os.path.dirname(__file__)#this method will gain his father path,所以，——currentdir是launch不是launch/****.launch.py
    ws_root = os.path.dirname(current_dir)#我们这样不会写死路径，这是根目录。




    #mavros启动
    mavros_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mavros'),
                'launch',
                'px4.launch'
            )
        ),
        launch_arguments={
            'fcu_url':'/dev/ttyACM0:921600',
            'gcs_url' : 'udp://@192.168.43.6'
        }.items()
    )
    #任务控制节点启动
    launch_task_controller_node = Node(
        package='offboard',
        executable='test_task_controller',
        name='offb_node',
        output = "screen"#和一位
    )
    delayed_task_controller= TimerAction(
        period=3.0,
        actions = [launch_task_controller_node]
    )
    event_handler_tc_node = RegisterEventHandler(
        OnIncludeLaunchDescription(
            matcher=mavros_launch,
            on_include=[delayed_task_controller]
        )
    )

    return LaunchDescription([
        livox_launch,
        delayed_fast_lio,
        event_handler_otp_node,
        event_handler_mavros,
        event_handler_tc_node,
    ])

# 总结：ai说我这个不太完美，因为我只是固定延迟了，没有去监听他是否真的运行起来了
