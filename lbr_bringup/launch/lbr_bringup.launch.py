from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Evaluate frequently used variables
    model = LaunchConfiguration("model").perform(context)

    # Load robot description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("lbr_description"),
                    "urdf/{}/{}.urdf.xacro".format(model, model),
                ]
            ),
            " ",
            "robot_name:=",
            LaunchConfiguration("robot_name"),
            " ",
            "sim:=",
            LaunchConfiguration("sim"),
        ]
    )

    # Load LBR FRI ROS2
    lbr_spinner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("lbr_fri_ros2"), "launch", "lbr_spinner.launch.py"]
            )
        ),
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )

    # Load controls
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("lbr_bringup"), "launch", "lbr_control.launch.py"]
            )
        ),
        launch_arguments=[
            ("robot_description", robot_description_content),
            (
                "controller_configurations_package",
                LaunchConfiguration("controller_configurations_package"),
            ),
            (
                "controller_configurations",
                LaunchConfiguration("controller_configurations"),
            ),
            ("controller", LaunchConfiguration("controller")),
            ("sim", LaunchConfiguration("sim")),
        ],
    )

    # Gazebo simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("lbr_bringup"), "launch", "lbr_simulation.launch.py"]
            )
        ),
        launch_arguments=[("robot_name", LaunchConfiguration("robot_name"))],
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    # Move group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("lbr_moveit"), "launch", "lbr_move_group.launch.py"]
            )
        ),
        launch_arguments=[
            ("robot_description", robot_description_content),
            (
                "moveit_controller_configurations_package",
                LaunchConfiguration("moveit_controller_configurations_package"),
            ),
            (
                "moveit_controller_configurations",
                LaunchConfiguration("moveit_controller_configurations"),
            ),
            ("model", LaunchConfiguration("model")),
            ("sim", LaunchConfiguration("sim")),
        ],
    )

    return [lbr_spinner, simulation, control, move_group]


def generate_launch_description():

    # Launch arguments
    launch_args = []

    launch_args.append(
        DeclareLaunchArgument(
            name="model",
            default_value="iiwa7",
            description="Desired LBR model. Use model:=iiwa7/iiwa14/med7/med14.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="robot_name", default_value="lbr", description="Set robot name."
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="sim",
            default_value="true",
            description="Launch robot in simulation or on real setup.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="real_time",
            default_value="false",
            description="Will launch ros2_control_node with real-time priority.\n"
            "\tCurrently only supported on Linux. Requires user to set rtprio\n"
            "\tin /etc/security/limits.conf, see https://linux.die.net/man/5/limits.conf.\n"
            "\tE.g. <user> - rtprio 99.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="controller_configurations_package",
            default_value="lbr_bringup",
            description="Package that contains controller configurations.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="controller_configurations",
            default_value="config/lbr_controllers.yml",
            description="Relative path to controller configurations YAML file.\n"
            "\tNote that the joints in the controllers must be named according to the robot_name.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="controller",
            default_value="position_trajectory_controller",
            description="Robot controller.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="moveit_controller_configurations_package",
            default_value="lbr_moveit",
            description="Package that contains MoveIt! controller configurations.",
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            name="moveit_controller_configurations",
            default_value="config/lbr_controllers.yml",
            description="Relative path to MoveIt! controller configurations YAML file.\n"
            "\tNote that the joints in the controllers must be named according to the robot_name.\n"
            "\tThis file lists controllers that are loaded through the controller_configurations file.",
        )
    )

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
