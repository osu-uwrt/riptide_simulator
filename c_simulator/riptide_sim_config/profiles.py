"""Resolve profiles to a portable, content-addressed run description."""

from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import yaml


def share(name):
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(name))


def asset(value, parent):
    if not value:
        return ""
    if value.startswith("package://"):
        package, relative = value[10:].split("/", 1)
        path = share(package) / relative
    else:
        path = Path(value)
        if not path.is_absolute():
            path = Path(parent) / path
    if not path.exists():
        raise ValueError(f"Missing asset: {path}")
    return str(path.resolve())


def read(path):
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return value


def merge(a, b):
    result = deepcopy(a)
    for key, value in b.items():
        result[key] = (
            merge(result[key], value)
            if isinstance(value, dict) and isinstance(result.get(key), dict)
            else deepcopy(value)
        )
    return result


def version(value, path):
    if value.get("schema_version") != 1:
        raise ValueError(f"{path}: unsupported schema_version (expected 1)")


def selection(value):
    if not re.fullmatch(r"[A-Za-z0-9_]+", value):
        raise ValueError(f"Invalid profile name: {value!r}")
    return value


def behavior_path(value, parent):
    if not value:
        return ""
    file, symbol = value.rsplit(":", 1)
    if not symbol.isidentifier():
        raise ValueError(f"Invalid behavior symbol: {symbol}")
    return asset(file, parent) + ":" + symbol


def load_behavior(reference):
    """Load an explicitly selected installed behavior, including relative imports."""
    path, symbol = reference.rsplit(":", 1)
    name = "_riptide_behavior_" + hashlib.sha256(str(Path(path).parent).encode()).hexdigest()[:16]
    module_name = name + "." + Path(path).stem
    if module_name not in sys.modules:
        import types

        package = types.ModuleType(name)
        package.__path__ = [str(Path(path).parent)]
        sys.modules.setdefault(name, package)
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    return getattr(sys.modules[module_name], symbol)


def create_mechanisms(node, vehicle, config):
    reference = config.get("robot_behavior")
    if not reference:
        return object()
    return load_behavior(reference)(node, vehicle, config.get("equipment", {}))


def validate_frames(data):
    done, active = set(), set()

    def visit(key):
        if key in ("map", "world"):
            return
        if key.endswith("_frame"):
            key = key[:-6]
        if key in done:
            return
        if key in active:
            raise ValueError(f"Cycle in world frames: {key}")
        if key not in data:
            raise ValueError(f"Unknown world frame: {key}")
        active.add(key)
        visit(data[key]["parent"])
        for number in data[key].get("pose", {}).values():
            if not isinstance(number, (int, float)) or not math.isfinite(number):
                raise ValueError(f"Invalid pose in frame {key}")
        active.remove(key)
        done.add(key)

    for key in data:
        visit(key)


def finite_vector(value, count, field):
    if (
        not isinstance(value, list)
        or len(value) != count
        or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in value)
    ):
        raise ValueError(f"{field}: expected {count} finite values")


def unique(items, key, field):
    ids = [item[key] for item in items]
    if len(ids) != len(set(ids)):
        raise ValueError(f"{field}: duplicate {key}")


def validate_ui(ui):
    options = ui.get("run_options", [])
    unique(options, "key", "ui.run_options")
    for option in options:
        selection(option["key"])
        kind, default = option["type"], option["default"]
        if kind == "bool" and not isinstance(default, bool):
            raise ValueError("Boolean run option requires a boolean default")
        if kind == "choice" and default not in [c["value"] for c in option["choices"]]:
            raise ValueError("Choice run option default must be one of its choices")
        if kind == "number" and (
            not isinstance(default, (float, int))
            or not math.isfinite(default)
            or not option.get("min", -math.inf) <= default <= option.get("max", math.inf)
        ):
            raise ValueError("Numeric run option default is outside its bounds")
        if kind not in ("bool", "choice", "number"):
            raise ValueError(f"Unknown run option type: {kind}")


def resolve(
    robot="talos", year="2026", scenario="default", *, root=None, output=None, overrides=None
):
    root = Path(root) if root else share("c_simulator")
    robot, year, scenario = map(selection, (str(robot), str(year), str(scenario)))
    # Resolve the three selections first; a scenario cannot silently select another robot/year.
    rp = root / "robots" / robot / "robot.yaml"
    yp = root / "tasks" / year / "competition.yaml"
    for path, kind in ((rp, "robot"), (yp, "year")):
        if not path.is_file():
            raise ValueError(f"Unknown {kind}: {path.parent.name} ({path})")
    r, y = read(rp), read(yp)
    version(r, rp)
    version(y, yp)
    if str(r.get("id")) != robot or str(y.get("id")) != year:
        raise ValueError("Profile identity does not match its folder")
    sp = root / "scenarios" / (scenario + ".yaml")
    if not sp.is_file():
        sp = yp.parent / "scenarios" / (scenario + ".yaml")
    if not sp.is_file():
        raise ValueError(f"Unknown scenario {scenario!r} for year {year}")
    s = read(sp)
    version(s, sp)
    if "robot" in s or "year" in s:
        raise ValueError("A scenario cannot override robot/year selection")
    o = overrides or {}
    namespace = selection(o.get("namespace") or robot)

    def robot_file(key, override=None):
        return asset(o.get(override or key + "_config") or r[key], rp.parent)

    vehicle = read(robot_file("vehicle"))
    hydro = read(robot_file("hydrodynamics"))
    sensor_path = robot_file("sensors", "sensor_config")
    simulator = read(robot_file("simulator"))
    equipment = read(robot_file("equipment"))
    wp = asset(o.get("world_config") or s["world"], sp.parent)
    world = read(wp)
    version(world, wp)
    for key in ("length", "width", "depth", "water_density"):
        if (
            not isinstance(world.get(key), (int, float))
            or not math.isfinite(world[key])
            or world[key] <= 0
        ):
            raise ValueError(f"{wp}: {key} must be finite and positive")
    for key in ("water_level", "current_oscillation_frequency", "deck_height"):
        if not math.isfinite(world.get(key, 0)):
            raise ValueError(f"{wp}: invalid {key}")
    for key in ("current_velocity", "current_oscillation_amplitude"):
        if len(world[key]) != 3 or not all(math.isfinite(v) for v in world[key]):
            raise ValueError(f"{wp}: invalid {key}")

    # Water properties belong to the world, even when a robot file supplies older defaults.
    hydro.update(
        {
            k: world[k]
            for k in (
                "water_density",
                "water_level",
                "current_velocity",
                "current_oscillation_amplitude",
                "current_oscillation_frequency",
            )
        }
    )
    version(hydro, robot_file("hydrodynamics"))
    import numpy as np

    for key, size in [("rigid_body_inertia3x3", 3), ("added_mass6x6", 6), ("linear_damping6x6", 6)]:
        matrix = np.asarray(hydro[key], dtype=float).reshape(size, size)
        if (
            not np.isfinite(matrix).all()
            or not np.allclose(matrix, matrix.T)
            or np.linalg.eigvalsh(matrix).min() < (1e-12 if size == 3 else -1e-10)
        ):
            raise ValueError(f"{rp}: invalid symmetric inertia/damping matrix {key}")
    hydro["robot"] = namespace
    vehicle["sim_model"] = robot
    if r.get("adapter", "generic") not in ("generic", "uwrt"):
        raise ValueError(f"{rp}: adapter must be generic or uwrt")
    if (
        not math.isfinite(vehicle.get("mass", 0))
        or vehicle.get("mass", 0) <= 0
        or not vehicle.get("thrusters")
    ):
        raise ValueError(f"{rp}: positive mass and at least one thruster required")
    if len(hydro.get("thruster_efficiencies", [])) != len(vehicle["thrusters"]):
        raise ValueError(f"{rp}: expected one efficiency per thruster")
    for key in ("com", "base_link"):
        finite_vector(vehicle[key], 3, f"{rp}: {key}")
    for i, thruster in enumerate(vehicle["thrusters"]):
        finite_vector(thruster["pose"], 6, f"{rp}: thrusters[{i}].pose")
    for value in hydro["thruster_efficiencies"]:
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"{rp}: invalid thruster efficiency")
    unique(vehicle.get("cameras", []), "name", f"{rp}: camera mounts")
    for mount in vehicle.get("cameras", []):
        finite_vector(mount["pose"], 6, f"{rp}: camera mount")

    # The manifest selects cameras; the vehicle description supplies their physical mounts.
    cameras = r.get("cameras", [])
    names = [c["name"] for c in cameras]
    if len(names) != len(set(names)):
        raise ValueError(f"{rp}: duplicate camera name")
    mounts = {c["name"]: c for c in vehicle.get("cameras", [])}
    resolved_cameras = []
    for c in cameras:
        c = deepcopy(c)
        selection(c["name"])
        if c.get("truth_tf_owner", "viewer") not in ("physics", "viewer"):
            raise ValueError(f"{rp}: invalid truth_tf_owner")
        if "intrinsics" in c:
            k = c["intrinsics"]
            for key in ("width", "height", "fx", "fy", "rate"):
                if not math.isfinite(k[key]) or k[key] <= 0:
                    raise ValueError(f"{rp}: camera {c['name']} invalid {key}")
            if any(not math.isfinite(k[key]) for key in ("cx", "cy")):
                raise ValueError(f"{rp}: invalid principal point")
        if c["name"] not in mounts:
            raise ValueError(f'{rp}: no mount for camera {c["name"]}')
        for key in ("config", "calibration"):
            if c.get(key):
                c[key] = asset(c[key], rp.parent)
        resolved_cameras.append(c)
    vehicle["sim_cameras"] = resolved_cameras
    vehicle["sim_adapter"] = r.get("adapter", "generic")
    vehicle["sim_start_pose"] = s.get("start_pose", [0.0, 0.0, -1.0, 0.0, 0.0, 0.0])
    finite_vector(vehicle["sim_start_pose"], 6, f"{sp}: start_pose")
    vehicle["sim_enabled_sensors"] = [
        k for k in ("imu", "dvl", "depth", "fog", "acoustics") if k in vehicle
    ]

    # Keep the C++ config reader satisfied without enabling absent sensor streams.
    for key in ("imu", "dvl", "depth"):
        vehicle.setdefault(
            key,
            dict(
                pose=[0.0] * 6,
                rate=50.0,
                sigma=0.0,
                yaw_drift=0.0,
                sigma_accel=0.0,
                sigma_omega=0.0,
                sigma_angle=0.0,
            ),
        )
    vehicle.setdefault("acoustics", dict(speed_of_sound=1480.0))
    simulator.setdefault("acoustics", {"fake_pinger": {"pose": [0.0] * 6}})
    enabled = bool(s.get("tasks", True)) and str(o.get("with_tasks", "true")).lower() not in (
        "false",
        "0",
    )
    if enabled:
        missing = set(y.get("required_capabilities", [])) - set(r.get("capabilities", {}))
        if missing:
            raise ValueError(
                f"Robot {robot} cannot run {year}/{scenario}; missing capabilities: {sorted(missing)}. Select scenario:=empty_pool or with_tasks:=false."
            )

    def year_data(key, default):
        return read(asset(y[key], yp.parent)) if key in y else deepcopy(default)

    # Only equipment requested by this year becomes part of its task/contact configuration.
    task = (
        merge(
            year_data("config", {}),
            {key: equipment[key] for key in y.get("equipment_sections", []) if key in equipment},
        )
        if enabled
        else {}
    )
    if o.get("task_config") and enabled:
        task = read(asset(o["task_config"], root))
    if enabled:
        task = merge(task, s.get("task_overrides", {}))
    task["world"] = world
    task["scoring_rules"] = (
        merge(year_data("scoring", {}), s.get("scoring_overrides", {})) if enabled else {}
    )
    task["ui"] = (
        deepcopy(y.get("ui", {}))
        if enabled
        else {
            "title": "Free simulation",
            "run_options": [],
            "focus": ["Course", "Vehicle"],
            "demo_targets": [],
        }
    )
    for option in task["ui"].get("run_options", []):
        if option["key"] in s.get("run_defaults", {}):
            option["default"] = s["run_defaults"][option["key"]]
    validate_ui(task["ui"])
    task["scoring_enabled"] = enabled
    task["robot_behavior"] = behavior_path(r.get("behavior"), rp.parent)
    task["equipment"] = equipment
    task["mechanism_controls"] = deepcopy(r.get("controls", []))
    task["robot_collision"] = asset(r["collision"], rp.parent)
    task["robot_model"] = asset(r["model"], rp.parent)
    mapping = year_data("mapping", {}) if enabled else {}
    if o.get("mapping_config"):
        mapping = read(asset(o["mapping_config"], root))
    entries = [
        v["ros__parameters"]["init_data"]
        for v in mapping.values()
        if isinstance(v, dict) and "init_data" in v.get("ros__parameters", {})
    ]
    if len(entries) > 1:
        selected = (
            mapping.get(f"/{robot}/riptide_mapping2", {})
            .get("ros__parameters", {})
            .get("init_data")
        )
        if selected is None:
            raise ValueError("Ambiguous mapping: select one course frame graph")
        entries = [selected]
    data = deepcopy(entries[0]) if entries else {}
    validate_frames(data)
    if enabled:
        missing_frames = set(y.get("required_frames", [])) - set(data)
        if missing_frames:
            raise ValueError(f"Missing task frames: {sorted(missing_frames)}")

    # Export one course under the instance namespace for existing UWRT mapping consumers.
    parameters = next(
        (
            deepcopy(v["ros__parameters"])
            for v in mapping.values()
            if isinstance(v, dict) and v.get("ros__parameters", {}).get("init_data") == data
        ),
        {},
    )
    parameters["init_data"] = data
    globals_ = {k: v for k, v in mapping.items() if k.startswith("/**/")}
    mapping = {
        **globals_,
        f"/{namespace}/riptide_mapping2": {"ros__parameters": parameters},
        "/**/zed_faker": mapping.get(
            "/**/zed_faker",
            {"ros__parameters": {"config_frame": "tag", "map_origin_pool": [0.0, 0.0, 0.0]}},
        ),
    }
    markers = year_data("markers", {}) if enabled else {}
    markers.setdefault("/**/marker_publisher", {"ros__parameters": {"markers": {}}})
    for m in markers["/**/marker_publisher"]["ros__parameters"]["markers"].values():
        frame = m.get("frame", "map").removesuffix("_frame")
        if frame not in ("map", "world") and frame not in data:
            raise ValueError(f"Unknown marker frame: {frame}")
    scene = year_data("scene", {}) if enabled else {"objects": {}, "april_tag": {"visible": False}}
    if o.get("scene_config"):
        scene = read(asset(o["scene_config"], root))
    scene["robot"] = {
        "model": {"riptide_mesh": task["robot_model"]},
        "collision": task["robot_collision"],
    }
    scene["world"] = world
    scene["entities"] = deepcopy(world.get("entities", [])) + scene.get("entities", [])
    unique(scene["entities"], "id", "scene.entities")
    for entity in scene["entities"]:
        selection(entity["id"])
        finite_vector(entity.get("pose", [0.0] * 6), 6, "entity.pose")
        finite_vector(entity["size"], 3, "entity.size")
        if any(v <= 0 for v in entity["size"]):
            raise ValueError("Entity sizes must be positive")
        if entity.get("frame", "map").removesuffix("_frame") not in ("map", "world", *data):
            raise ValueError("Unknown entity frame")
        if entity.get("mesh"):
            entity["mesh"] = asset(entity["mesh"], yp.parent)
    task["behavior"] = behavior_path(y.get("behavior"), yp.parent) if enabled else ""
    task["year"] = year
    if enabled and y.get("validator"):
        load_behavior(behavior_path(y["validator"], yp.parent))(task)
    # Scenario variations stay within task/rules ownership; robot physics remains
    # in the robot profile and the world remains authoritative for every consumer.
    task = merge(task, {key: s[key] for key in ("run_defaults",) if key in s})
    viewer = {key: asset(value, rp.parent) for key, value in r.get("viewer", {}).items()}
    summary = dict(
        schema_version=1,
        resolver_revision=2,
        namespace=namespace,
        robot=robot,
        year=year,
        scenario=scenario,
        enabled=enabled,
        adapter=r.get("adapter", "generic"),
        world=world,
        viewer=viewer,
        mesh_root=asset(y.get("mesh_root", ""), yp.parent),
        robot_behavior=task["robot_behavior"],
        capabilities=r.get("capabilities", {}),
    )
    runtime = dict(
        physics_step=0.002,
        real_time_factor=1.0,
        clock_publish_rate=500.0,
        random_seed=7,
        sensor_noise=True,
        collisions=True,
    )
    runtime.update(s.get("runtime", {}))
    runtime.update({k: o[k] for k in runtime if k in o})
    summary["runtime"] = runtime
    task["runtime"] = runtime
    summary["behavior_hashes"] = {
        str(file): hashlib.sha256(file.read_bytes()).hexdigest()
        for parent in (rp.parent / "behavior", yp.parent / "behavior")
        for file in sorted(parent.glob("*.py"))
    }
    documents = dict(
        vehicle=vehicle,
        hydrodynamics=hydro,
        simulator=simulator,
        task=task,
        mapping=mapping,
        markers=markers,
        scene=scene,
        sensors=read(sensor_path),
        selection=summary,
    )

    # Every consumer gets the same snapshot. The ID excludes its own output fields.
    digest = hashlib.sha256(json.dumps(documents, sort_keys=True).encode()).hexdigest()[:20]
    task["config_id"] = digest
    summary["config_id"] = digest
    cache = (
        Path(output)
        if output
        else Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros"))) / "riptide_simulator/runs"
    )
    cache.mkdir(parents=True, exist_ok=True)
    destination = cache / digest
    if not destination.is_dir():
        # Publish the directory only after every file is complete; concurrent launches may reuse it.
        temp = Path(tempfile.mkdtemp(prefix=".resolve-", dir=cache))
        try:
            for key, value in documents.items():
                (temp / (key + ".yaml")).write_text(yaml.safe_dump(value, sort_keys=False))
            try:
                temp.rename(destination)
            except OSError:
                if not destination.is_dir():
                    raise
        finally:
            import shutil

            if temp.exists():
                shutil.rmtree(temp)
    return destination
