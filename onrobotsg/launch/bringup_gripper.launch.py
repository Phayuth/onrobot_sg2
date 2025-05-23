from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    la = []
    la.append(
        DeclareLaunchArgument(
            "ip",
            default_value="192.168.0.137",
            description="IP Address to Gripper Compute Box",
        ),
    )
    la.append(
        DeclareLaunchArgument(
            "port",
            default_value="502",
            description="Port ID.",
        )
    )
    la.append(
        DeclareLaunchArgument(
            "model_id",
            default_value="4",
            description="Onrobot SG Model ID: 1:None, 2:SG-a-H(Flower-Like w/o White Tip), 3:SG-a-S(Flower-Like w/ White Tip), 4:SG-b-H(Claw-Like)",
            choices=["1", "2", "3", "4"],
        )
    )
    la.append(
        DeclareLaunchArgument(
            "gentle",
            default_value="true",
            description="Gripping speed is reduced at 12.5mm before the specified target width",
        )
    )
    la.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware instead of real hardware",
        )
    )

    ld = []
    ld.append(
        Node(
            package="onrobotsg",
            executable="onrobotsg_service",
            name="onrobotsg_node",
            parameters=[
                {"ip": LaunchConfiguration("ip")},
                {"port": LaunchConfiguration("port")},
                {"model_id": LaunchConfiguration("model_id")},
                {"gentle": LaunchConfiguration("gentle")},
                {"use_fake_hardware": LaunchConfiguration("use_fake_hardware")},
            ],
            output="screen",
        ),
    )

    return LaunchDescription(la + ld)
