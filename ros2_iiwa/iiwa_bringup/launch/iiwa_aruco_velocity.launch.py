from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os

import shutil

def _resolve_sim_command(context):
    # Choose available simulator binary: prefer 'gz', fallback to 'ign' or classic 'gazebo'
    candidates = [
        ['gz', 'sim'],           # Gazebo (Fortress+, Harmonic)
        ['ign', 'gazebo'],       # Ignition Gazebo
        ['gazebo']               # Classic Gazebo
    ]
    for cmd in candidates:
        exe = cmd[0]
        if shutil.which(exe) is not None:
            return cmd
    return None


def generate_launch_description():
    # Minimal Gazebo launch to load the ArUco world for Homework 2a.

    iiwa_desc_share = get_package_share_directory('iiwa_description')
    models_dir = os.path.join(iiwa_desc_share, 'gazebo', 'models')
    world_path = os.path.join(iiwa_desc_share, 'gazebo', 'worlds', 'iiwa_aruco.world')

    # Ensure Gazebo can resolve model://arucotag
    current_resources = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = f"{models_dir}:{current_resources}" if current_resources else models_dir

    def launch_setup(context, *args, **kwargs):
        sim_cmd = _resolve_sim_command(context)
        if sim_cmd is None:
            raise RuntimeError("No Gazebo simulator binary found. Install 'gz', 'ign', or 'gazebo'.")
        cmd = sim_cmd + [world_path]
        return [
            SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
            ExecuteProcess(cmd=cmd, output='screen')
        ]

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
