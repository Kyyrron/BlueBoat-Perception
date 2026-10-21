"""Start BlueBoat-Perception.

    ros2 launch blueboat_perception perception_launch.py \
        [mode:=normal|dataset] [dataset_name:=<YYYY_MM_DD-HH_MM_SS>-<note>] \
        [world_dir:=<world folder>] [data_dir:=<root>]

* ``mode:=normal`` (default): the node only.
* ``mode:=dataset``: the node also writes a dataset. This file claims the
  dataset folder (``<root>/data/Dataset_data/<dataset_name>``, ``-2`` ...
  on a clash; a time-stamped name when none is given) and passes it to
  the node. When ``world_dir`` is set (a simulated personalized-world
  run) it also starts an **archival** ``ros2 bag record`` of the
  simulator ground truth into ``<dataset>/ground_truth_bag`` -- a separate
  process, from which ``blueboat-perception-offline --world
  --ground-truth-bag`` re-labels offline. In that same case the node
  itself subscribes to the two ground-truth topics, for the dataset labels
  only (``ros/ground_truth.py`` -> ``core/sim_labels.py``); detections and
  records never see them.

MCS starts this for Gazebo runs of a personalized world (the chosen
session mode) and for every real-robot launch (normal).
"""

import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GROUND_TRUTH_TOPICS = ["/side_scan_sonar/ground_truth/contacts",
                       "/side_scan_sonar/ground_truth/sensor"]


def _setup(context):
    from blueboat_perception.core.dataroot import (claim_dir, data_root,
                                                   dataset_root, safe_name)
    mode = LaunchConfiguration("mode").perform(context).strip() or "normal"
    name = LaunchConfiguration("dataset_name").perform(context).strip()
    world_dir = LaunchConfiguration("world_dir").perform(context).strip()
    data_dir = LaunchConfiguration("data_dir").perform(context).strip()
    params = {"mode": mode, "dataset_name": name, "world_dir": world_dir,
              "data_dir": data_dir}
    actions = []
    if mode == "dataset":
        if not name:
            name = time.strftime("%Y_%m_%d-%H_%M_%S") + "-dataset"
        d = claim_dir(dataset_root(data_root(data_dir)), safe_name(name))
        params["dataset_name"] = d.name
        params["dataset_path"] = str(d)
        actions.append(LogInfo(msg=f"[perception] dataset folder: {d}"))
        if world_dir:
            actions.append(ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", str(Path(d) / "ground_truth_bag"),
                     "--topics", *GROUND_TRUTH_TOPICS],
                name="perception_ground_truth_recorder", output="screen"))
        else:
            actions.append(LogInfo(msg="[perception] no world_dir: not a simulated "
                                       "world run, no ground-truth recording"))
    config = str(Path(get_package_share_directory("blueboat_perception"))
                 / "config" / "perception.yaml")
    actions.append(Node(package="blueboat_perception", executable="perception_node",
                        name="perception", output="screen",
                        parameters=[config, params]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="normal",
                              description="normal | dataset"),
        DeclareLaunchArgument("dataset_name", default_value="",
                              description="dataset folder name (dataset mode)"),
        DeclareLaunchArgument("world_dir", default_value="",
                              description="the simulated world folder (recorded in the "
                                          "dataset manifest; enables the ground-truth bag)"),
        DeclareLaunchArgument("data_dir", default_value="",
                              description="data root override (else $BLUEBOAT_DATA_DIR, "
                                          "then the workspace)"),
        OpaqueFunction(function=_setup),
    ])
