"""Launch configuration plumbing shared by bringup, physics, and rendering."""

from pathlib import Path
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from .profiles import resolve, read


def arguments():
    return [
        DeclareLaunchArgument("robot", default_value="talos"),
        DeclareLaunchArgument("namespace", default_value=LaunchConfiguration("robot")),
        DeclareLaunchArgument("year", default_value="2026"),
        DeclareLaunchArgument("scenario", default_value="default"),
        DeclareLaunchArgument("resolved_config", default_value=""),
    ] + [
        DeclareLaunchArgument(key, default_value="")
        for key in (
            "world_config",
            "mapping_config",
            "task_config",
            "hydrodynamics_config",
            "sensor_config",
            "scene_config",
        )
    ]


def run(context):
    values = context.launch_configurations
    # Nested launch files reuse the parent snapshot instead of resolving different defaults.
    selected = values.get("resolved_config", "")
    if not selected:
        selected = str(
            resolve(
                values.get("robot", "talos"),
                values.get("year", "2026"),
                values.get("scenario", "default"),
                overrides={
                    k: values[k]
                    for k in (
                        "namespace",
                        "mapping_config",
                        "task_config",
                        "hydrodynamics_config",
                        "sensor_config",
                        "scene_config",
                        "world_config",
                        "with_tasks",
                        "physics_step",
                        "real_time_factor",
                        "clock_publish_rate",
                        "random_seed",
                        "sensor_noise",
                        "collisions",
                    )
                    if values.get(k)
                },
            )
        )
        values["resolved_config"] = selected
    path = Path(selected)
    meta = read(path / "selection.yaml")
    if meta["robot"] != values.get("robot", "talos") or meta["year"] != values.get("year", "2026"):
        raise ValueError("Resolved configuration does not match robot/year selections")
    if meta["scenario"] != values.get("scenario", "default") or meta["namespace"] != values.get(
        "namespace", meta["robot"]
    ):
        raise ValueError("Resolved configuration does not match scenario/namespace selections")
    for key, value in meta["runtime"].items():
        values.setdefault(key, str(value).lower() if isinstance(value, bool) else str(value))
    return path, meta


def prepare(context):
    path, meta = run(context)
    optional = [
        SetLaunchConfiguration(key, "true" if meta["adapter"] == "uwrt" else "false")
        for key in ("with_bringup", "with_rviz", "with_apriltag")
        if context.launch_configurations.get(key, "auto") == "auto"
    ]
    return optional + [
        SetLaunchConfiguration("resolved_config", str(path)),
        SetLaunchConfiguration("simulator_mapping_config", str(path / "mapping.yaml")),
    ]
