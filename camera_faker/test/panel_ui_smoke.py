#!/usr/bin/env python3
"""X11 display-backed control test with mocked estimation and controller services.

Requires python3-xlib and an otherwise idle display. No hardware is launched.
Run with an isolated ROS_DOMAIN_ID and RMW_IMPLEMENTATION=rmw_fastrtps_cpp.
"""
import os
import math
from pathlib import Path
import signal
import sys
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.task import Future
from riptide_msgs2.action import ExecuteTree
from riptide_msgs2.srv import ListTrees
from geometry_msgs.msg import TransformStamped
from riptide_msgs2.msg import ControllerCommand, KillSwitchReport
from std_srvs.srv import SetBool
from riptide_sim_config.profiles import resolve, read
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from Xlib import X, XK, display, protocol
from Xlib.ext import xtest


def main():
    if os.environ.get("ROS_DOMAIN_ID", "0") == "0":
        raise RuntimeError("Use an isolated ROS_DOMAIN_ID")
    rclpy.init()
    robot = f"operator_ui_{os.getpid()}"
    node = rclpy.create_node("operator_ui_test", namespace=robot)
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)

    def transform(parent, child, xyz=(0., 0., 0.)):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.w = 1.
        return t

    # Match the profile's camera mount, not an identity transform inside the hull.
    vehicle = read(resolve("talos", overrides={"namespace": robot}) / "vehicle.yaml")
    camera = next(c for c in vehicle["cameras"] if c["name"] == "ffc")
    mount = transform(f"simulator/{robot}/base_link", f"simulator/{robot}/ffc_camera_link",
                      tuple(float(a-b) for a, b in zip(camera["pose"][:3], vehicle["base_link"])))
    r, p, y = (a / 2 for a in camera["pose"][3:])
    cr, cp, cy, sr, sp, sy = math.cos(r), math.cos(p), math.cos(y), math.sin(r), math.sin(p), math.sin(y)
    q = mount.transform.rotation
    q.w, q.x = cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy
    q.y, q.z = cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy
    static.sendTransform([transform("world", "map"), mount])
    poses = [transform("map", f"{robot}/base_link", (3., -2., -.75)),
             transform("map", f"simulator/{robot}/base_link", (3., -2., -.75))]
    commands, angular, kills = [], [], []
    node.create_subscription(ControllerCommand, "controller/linear", commands.append, 10)
    node.create_subscription(ControllerCommand, "controller/angular", angular.append, 10)
    node.create_subscription(KillSwitchReport, "command/software_kill", kills.append, 10)

    def teleop(request, response):
        assert not request.data
        response.success = True
        return response

    node.create_service(SetBool, "setTeleop", teleop)
    mission = {"goal": None, "future": None, "starts": 0, "stops": 0}

    def list_trees(request, response):
        response.trees = ["/test/ControlSmoke.xml"]
        return response

    async def execute(goal):
        return await mission["future"]

    def accepted(goal):
        mission["goal"], mission["future"] = goal, Future()
        mission["starts"] += 1
        goal.execute()

    def feedback():
        goal = mission["goal"]
        if goal is None:
            return
        if goal.is_cancel_requested:
            goal.canceled()
            mission["future"].set_result(ExecuteTree.Result())
            mission["goal"] = None
            mission["stops"] += 1
        else:
            message = ExecuteTree.Feedback()
            message.stack.stack = ["Root", "HoldDepth"]
            goal.publish_feedback(message)

    node.create_service(ListTrees, "autonomy/list_trees", list_trees)
    action_server = ActionServer(node, ExecuteTree, "autonomy/run_tree", execute,
                                 handle_accepted_callback=accepted,
                                 cancel_callback=lambda goal: CancelResponse.ACCEPT)
    node.create_timer(.1, feedback)
    speed_state = {"value": 2.0, "writes": [], "syncs": 0, "resets": 0}
    if os.environ.get("PANEL_VIEW_TOOLS_ONLY"):
        from rcl_interfaces.srv import GetParameters, SetParameters
        from std_srvs.srv import Trigger
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray, Marker
        from riptide_msgs2.msg import ActuatorStatus
        def sync_sim(request, response):
            speed_state["syncs"] += 1
            response.success, response.message = True, "Synced to estimate"
            return response
        def reset_sim(request, response):
            speed_state["resets"] += 1
            response.success, response.message = True, "Reset to start"
            return response
        node.create_service(Trigger, "sync_sim_to_estimate", sync_sim)
        node.create_service(Trigger, "reset_sim_to_start", reset_sim)
        score_pub = node.create_publisher(String, "simulator/run_score", 10)
        event_pub = node.create_publisher(String, "simulator/task_events", 10)
        magnet_pub = node.create_publisher(MarkerArray, "simulator/magnet_lights", 10)
        actuator_pub = node.create_publisher(ActuatorStatus, "state/actuator/status", 10)
        def telemetry():
            score_pub.publish(String(data='{"running":false,"elapsed":12,"total":25,"rows":[]}'))
            light = Marker(ns="magnet_target1", action=Marker.ADD)
            light.color.g, light.color.a = 1.0, 1.0
            magnet_pub.publish(MarkerArray(markers=[light]))
            actuator_pub.publish(ActuatorStatus(torpedo_available_count=2, dropper_available_count=2))
        node.create_timer(.1, telemetry)
        from rcl_interfaces.msg import ParameterValue, ParameterType, SetParametersResult
        def get_speed(request, response):
            assert request.names == ["real_time_factor"]
            response.values = [ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                              double_value=speed_state["value"])]
            return response
        def set_speed(request, response):
            assert request.parameters[0].name == "real_time_factor"
            speed_state["value"] = request.parameters[0].value.double_value
            speed_state["writes"].append(speed_state["value"])
            response.results = [SetParametersResult(successful=True)]
            return response
        node.create_service(GetParameters, "physics_simulator/get_parameters", get_speed)
        node.create_service(SetParameters, "physics_simulator/set_parameters", set_speed)

    d = display.Display()
    for button in (1, 2, 3):
        xtest.fake_input(d, X.ButtonRelease, button)
    d.sync()
    root = d.screen().root

    def windows(w):
        for child in w.query_tree().children:
            if child.get_wm_name() == "Riptide | RoboSub Pool":
                yield child
            yield from windows(child)

    existing = {w.id for w in windows(root)}
    output = Path("/tmp/riptide-panel-ui")
    output.mkdir(exist_ok=True)
    log = (output / "viewer.log").open("w")
    process = subprocess.Popen([
        "ros2", "launch", "camera_faker", "pool_viewer.launch.py", "robot:=talos",
        f"namespace:={robot}", "use_sim_time:=false", "with_apriltag:=false",
        "with_rviz:=" + ("true" if os.environ.get("PANEL_RVIZ_ONLY") else "false"),
    ], stdout=log, stderr=log, start_new_session=True)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            for pose in poses:
                pose.header.stamp = stamp
            dynamic.sendTransform(poses)
            rclpy.spin_once(node, timeout_sec=.01)
            assert process.poll() is None, (output / "viewer.log").read_text()

    try:
        window = None
        deadline = time.monotonic() + 20
        while window is None and time.monotonic() < deadline:
            spin(.1)
            window = next((w for w in windows(root) if w.id not in existing), None)
        assert window is not None, "Viewer window not found"
        while window.get_attributes().map_state != X.IsViewable and time.monotonic() < deadline:
            spin(.1)
        assert window.get_attributes().map_state == X.IsViewable, "Viewer window not mapped"
        window.configure(stack_mode=X.Above)
        root.send_event(protocol.event.ClientMessage(
            window=window, client_type=d.intern_atom("_NET_ACTIVE_WINDOW"),
            data=(32, [1, X.CurrentTime, 0, 0, 0])),
            event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
        window.set_input_focus(X.RevertToParent, X.CurrentTime)
        d.sync()
        spin(2.5)

        def move(x, y):
            offset = root.translate_coords(window, 0, 0)
            xtest.fake_input(d, X.MotionNotify, x=int(offset.x + x), y=int(offset.y + y))
            d.sync()

        def click(x, y):
            window.set_input_focus(X.RevertToParent, X.CurrentTime)
            d.sync()
            move(x, y)
            spin(.3)
            xtest.fake_input(d, X.ButtonPress, 1)
            d.sync()
            spin(.3)
            xtest.fake_input(d, X.ButtonRelease, 1)
            d.sync()
            spin(.35)

        def screenshot(name):
            geometry = window.get_geometry()
            shot = window.get_image(0, 0, geometry.width, geometry.height, X.ZPixmap, 0xffffffff)
            pixels = np.frombuffer(shot.data, dtype=np.uint8).reshape(geometry.height, geometry.width, 4)[:, :, :3]
            cv2.imwrite(str(output / (name + ".png")), pixels)
            return pixels

        assert not kills and not commands, "Viewer published controls before interaction"
        screenshot("initial")
        if os.environ.get("PANEL_CAPTURE_ONLY"):
            return
        if os.environ.get("PANEL_RVIZ_ONLY"):
            # The viewer's control publishers/subscriptions must never be constructed.
            from riptide_msgs2.msg import ActuatorStatus, MappingTargetInfo
            actuator_probe = node.create_publisher(ActuatorStatus, "state/actuator/status", 10)
            mapping_probe = node.create_publisher(MappingTargetInfo, "state/mapping", 10)
            spin(.3)
            assert actuator_probe.get_subscription_count() == 0
            assert mapping_probe.get_subscription_count() == 0
            assert node.count_publishers("command/software_kill") == 0
            assert node.count_publishers("controller/linear") == 0
            assert not kills and not commands
            screenshot("rviz-viewer-tools-only")
            print("PASS: RViz launch suppresses robot-control providers; simulator tools remain")
            return
        # Normalize widths for the coordinate-based interaction checks. The capture
        # phase above keeps the real defaults for visual review.
        for x, delta in ((455, -113), (1025, 37)):
            move(x, 250)
            spin(.15)
            xtest.fake_input(d, X.ButtonPress, 1)
            d.sync()
            spin(.15)
            move(x + delta, 250)
            spin(.2)
            xtest.fake_input(d, X.ButtonRelease, 1)
            d.sync()
            spin(.2)
        if os.environ.get("PANEL_VIEW_TOOLS_ONLY"):
            before = screenshot("viewer-lighting-before")
            assert speed_state["writes"] == []
            click(678,78)
            screenshot("view-settings-menu")
            click(712,253)  # Observer shadows off.
            click(828,329) # Exposure up.
            click(808,366) # Direct light up.
            click(816,405) # Ambient light up.
            click(788,290)  # Select Indoor viewer lighting.
            for key in ("Down", "Return"):
                code = d.keysym_to_keycode(XK.string_to_keysym(key))
                xtest.fake_input(d, X.KeyPress, code)
                xtest.fake_input(d, X.KeyRelease, code)
                d.sync()
                spin(.15)
            click(1100,50)
            screenshot("viewer-lighting-shadows-off")
            click(678,78)
            click(712,253) # Enable observer shadows with the changed lighting mode.
            click(1100,50)
            spin(.5)
            after = screenshot("viewer-lighting-after")
            def diff(region):
                return np.abs(before[region].astype(float)-after[region].astype(float)).mean()
            assert diff(np.s_[200:850,360:1045]) > 10, "Viewer lighting did not change"
            for sensor in [np.s_[200:320,1120:1400],np.s_[540:660,1120:1400]]:
                assert diff(sensor) < 6, "View lighting changed FFC/DFC output"
            click(678,78)
            click(778,444) # Reset only View lighting.
            click(1100,50)
            spin(.5)
            reset_light = screenshot("viewer-lighting-reset")
            assert np.abs(before[200:850,360:1045].astype(float)-reset_light[200:850,360:1045].astype(float)).mean() < 6
            # Sterile is ambient-only and must not alter either sensor camera.
            click(678,78)
            click(788,290)
            click(750,400) # Sterile
            click(1100,50)
            spin(.5)
            sterile = screenshot("viewer-lighting-sterile")
            assert sterile[200:850,360:1045].mean() < before[200:850,360:1045].mean() - 5
            for sensor in [np.s_[200:320,1120:1400], np.s_[540:660,1120:1400]]:
                assert np.abs(before[sensor].astype(float)-sterile[sensor].astype(float)).mean() < 6
            click(678,78)
            click(778,444)
            click(1100,50)
            click(545,78)
            screenshot("simulation-settings-menu")
            click(633,220)
            spin(.7)
            assert speed_state["writes"] == [0.0]
            screenshot("simulation-paused-retains-speed")
            click(633,220) # Same button resumes the retained 2x rate.
            spin(.7)
            assert speed_state["writes"] == [0.0, 2.0]
            # Numeric editor applies once; opening/reading settings stays passive.
            click(550,180)
            ctrl = d.keysym_to_keycode(XK.string_to_keysym("Control_L"))
            akey = d.keysym_to_keycode(XK.string_to_keysym("a"))
            xtest.fake_input(d,X.KeyPress,ctrl)
            xtest.fake_input(d,X.KeyPress,akey)
            xtest.fake_input(d,X.KeyRelease,akey)
            xtest.fake_input(d,X.KeyRelease,ctrl)
            two = d.keysym_to_keycode(XK.string_to_keysym("4"))
            xtest.fake_input(d,X.KeyPress,two)
            xtest.fake_input(d,X.KeyRelease,two)
            d.sync()
            spin(.3)
            click(538,220)
            spin(.7)
            assert speed_state["writes"] == [0.0,2.0,4.0], speed_state
            click(633,220)
            spin(.7)
            screenshot("simulation-paused-4x")
            click(633,220)
            spin(.7)
            assert speed_state["writes"] == [0.0,2.0,4.0,0.0,4.0]
            click(558,323)
            spin(.3)
            click(698,323)
            spin(.3)
            screenshot("simulation-sync-reset")
            assert speed_state["syncs"] == speed_state["resets"] == 1, speed_state
            click(1100,50)
            event_pub.publish(String(data='{"kind":"magnet","result":"green","target":"magnet_target1"}'))
            spin(.2)
            click(990,116)
            screenshot("run-task-feedback")
            # Shrinking the app keeps the full scorecard window within its work area.
            window.configure(width=1000, height=640)
            d.sync()
            spin(.5)
            screenshot("run-tracking-small-window")
            print("PASS: viewer lighting/reset/Sterile preserve cameras; Pause/Resume, Sync/Reset, bounded run window")
            return
        if os.environ.get("PANEL_TOOLS_ONLY"):
            click(140, 162)  # Collapse Motion to give each tools panel room.
            click(140, 209)
            screenshot("mapping-panel")
            click(140, 209)
            click(140, 256)
            screenshot("actuator-panel")
            click(140, 256)
            click(990, 116)  # Run tracking is now a toolbar tool.
            screenshot("run-scorecard")
            assert not kills and not commands, "Browsing tools must remain passive"
            print("PASS: mapping, actuator, run panels and independent scorecard; captures:", output)
            return
        click(250, 95) # Enable
        spin(.2)
        screenshot("enabled")
        assert kills and not kills[-1].switch_asserting_kill
        click(150, 249) # Select Position; no separate Command is needed for dragging.
        spin(.6)
        screenshot("hold")
        assert commands[-1].mode == ControllerCommand.POSITION
        assert abs(commands[-1].setpoint_vect.x - 3.) < .001
        assert abs(commands[-1].setpoint_vect.y + 2.) < .001

        def mouse_drag(x, y, dx, dy, button=1, capture=None):
            window.set_input_focus(X.RevertToParent, X.CurrentTime)
            d.sync()
            move(x, y)
            spin(.25)
            xtest.fake_input(d, X.ButtonPress, button)
            d.sync()
            spin(.25)
            for step in range(1, 9):
                move(x + dx * step / 8, y + dy * step / 8)
                spin(.12)
            if capture:
                screenshot(capture)
            xtest.fake_input(d, X.ButtonRelease, button)
            d.sync()
            spin(.2)

        if os.environ.get("PANEL_LAYOUT_ONLY"):
            def selected_orbit(pixels):
                text = pixels[106:126, 480:565].astype(int)
                return (text.min(axis=2) > 140) & (text.max(axis=2)-text.min(axis=2) < 50)

            def has_follow(pixels):
                return np.max(pixels[106:124,617:637,1]) > 180

            def focus_gold(pixels):
                blue,green,red = pixels[520:562,680:724].astype(int).transpose(2,0,1)
                return (red>blue+40) & (green>blue+30) & (red>green-10)

            move(1000,140)
            spin(.2)
            camera_commands=len(commands)
            initial=screenshot("layout-initial")
            def difference(a,b,region):
                return np.abs(a[region].astype(float)-b[region].astype(float)).mean()

            def option(y):
                click(678,78)
                click(712,y)
                click(1100,50)

            click(100,585) # Robot controls in Motion
            no_handles=screenshot("layout-handles-hidden")
            assert not np.any(np.all(no_handles[200:850,360:1045] == [255,145,80],axis=2))
            click(100,585)
            option(109) # Water
            no_water=screenshot("layout-no-water")
            option(146) # Pool walls
            no_walls=screenshot("layout-no-walls")
            region=np.s_[200:850,360:1045]
            assert difference(initial,no_water,region) > 15
            assert difference(no_water,no_walls,region) > 5
            for sensor in [np.s_[200:320,1120:1400],np.s_[540:660,1120:1400]]:
                assert difference(initial,no_walls,sensor) < 6, "Observer option changed sensor output"
            option(109)
            option(146)

            def arrow_span():
                pixels=screenshot("layout-gizmo-size")
                ys,xs=np.where(np.all(pixels[200:850,360:1045] == [110,215,90],axis=2))
                assert len(xs)>50
                return np.ptp(xs)

            near=arrow_span()
            move(1000,780)
            for _ in range(5):
                xtest.fake_input(d,X.ButtonPress,5)
                xtest.fake_input(d,X.ButtonRelease,5)
                d.sync()
                spin(.12)
            assert .5 < arrow_span()/near < .75
            for _ in range(5):
                xtest.fake_input(d,X.ButtonPress,4)
                xtest.fake_input(d,X.ButtonRelease,4)
                d.sync()
                spin(.12)
            label=selected_orbit(initial)
            assert has_follow(initial)
            mouse_drag(420,240,18,-8,button=3,capture="layout-pan-held")
            panned=screenshot("layout-panned")
            assert not has_follow(panned)
            assert np.array_equal(selected_orbit(panned),label), "Pan changed the orbit preset"
            assert np.count_nonzero(focus_gold(panned)) < 10, "Focus disk visible at rest"
            click(626,115) # Resume the retained Vehicle target.
            assert has_follow(screenshot("layout-follow-resumed"))
            move(970,800)
            spin(.3)
            code=d.keysym_to_keycode(XK.string_to_keysym("f"))
            xtest.fake_input(d,X.KeyPress,code)
            d.sync()
            spin(.15)
            xtest.fake_input(d,X.KeyRelease,code)
            d.sync()
            spin(.3)
            move(1000,140)
            focused=screenshot("layout-focused")
            assert not has_follow(focused)
            assert np.array_equal(selected_orbit(focused),label), "F changed the orbit preset"

            # Gate has free space at its orbit center, so depth cannot mask an
            # incorrectly visible focus disk while Follow is checked.
            click(536,116)
            click(536,201)
            move(1000,140)
            spin(.3)
            assert has_follow(screenshot("layout-gate-follow"))
            mouse_drag(420,240,18,-8,capture="layout-follow-rotating")
            assert np.count_nonzero(focus_gold(cv2.imread(str(output / "layout-follow-rotating.png")))) < 10
            click(626,115)
            mouse_drag(420,240,-18,8,capture="layout-free-rotating")
            assert np.count_nonzero(focus_gold(cv2.imread(str(output / "layout-free-rotating.png")))) > 20
            assert np.count_nonzero(focus_gold(screenshot("layout-focus-idle"))) < 10

            assert len(commands) == camera_commands, "Camera gesture commanded the robot"

            # Both edges retain their hit area while the grab bar is hidden.
            move(900,140)
            spin(.2)
            quiet=screenshot("layout-edge-idle")
            assert not np.any(np.all(quiet[200:850,1060:1065] == [190,200,70],axis=2))
            move(1062,250)
            spin(.2)
            hover=screenshot("layout-edge-hover")
            assert np.any(np.all(hover[200:850,1060:1065] == [190,200,70],axis=2))
            mouse_drag(1062,250,-70,0,capture="layout-cameras-wider")
            # Reverse the same drag after snapping closed, then release closed.
            move(992,250)
            spin(.3)
            xtest.fake_input(d,X.ButtonPress,1)
            d.sync()
            spin(.3)
            move(1262,250)
            spin(.3)
            closed=screenshot("layout-cameras-snap-held")
            assert np.max(closed[800,1452:1457,1]) > 180
            move(992,250)
            spin(.3)
            restored=screenshot("layout-cameras-reverse-held")
            assert np.max(restored[800,990:995,1]) > 180
            move(1262,250)
            spin(.3)
            xtest.fake_input(d,X.ButtonRelease,1)
            d.sync()
            spin(.3)
            mouse_drag(1454,250,-90,0,capture="layout-cameras-reopened")
            mouse_drag(1154,250,-92,0)
            move(900,140)
            spin(.2)
            screenshot("layout-restored")
            click(780,78)
            screenshot("layout-pool-panels-menu")
            click(1100,50)
            window.configure(width=1280,height=800)
            d.sync()
            spin(.5)
            screenshot("layout-narrow")
            print("PASS: pan/F retain orbit preset, focus disk obeys Follow, mirrored camera resize/snap/reopen, pool toolbar and narrow layout")
            return

        def drag_at(x, y, dx, dy):
            move(x, y)
            xtest.fake_input(d, X.ButtonPress, 1)
            d.sync()
            spin(.25)
            before = len(commands)
            for step in range(1, 9):
                move(x + dx * step / 8, y + dy * step / 8)
                spin(.06)
            xtest.fake_input(d, X.ButtonRelease, 1)
            d.sync()
            spin(.3)
            assert len(commands) > before, "Dragging target did not send commands"
            assert commands[-1].mode == ControllerCommand.POSITION

        def drag_handle(name, rgb, dx, dy):
            pixels = screenshot(name + "-before")
            crop = pixels[153:840, 342:1160]
            mask = np.all(crop == np.array(rgb[::-1], dtype=np.uint8), axis=2).astype(np.uint8)
            # The filled handle has a larger interior than its label or axis line.
            distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
            _, radius, _, center = cv2.minMaxLoc(distance)
            assert radius >= 3.5, f"{name} handle not found"
            x, y = np.array(center) + (342, 153)
            drag_at(x, y, dx, dy)
            screenshot(name + "-after")

        drag_handle("xy", (90, 235, 230), 25, -12)
        xy = commands[-1].setpoint_vect
        assert abs(xy.x - 3.) + abs(xy.y + 2.) > .01
        assert abs(xy.z + .75) < .001
        for name, color in [("x", (235, 75, 75)), ("y", (90, 215, 110))]:
            before_xyz = commands[-1].setpoint_vect
            drag_handle(name, color, 18, -12)
            after_xyz = commands[-1].setpoint_vect
            assert abs(after_xyz.x-before_xyz.x) + abs(after_xyz.y-before_xyz.y) > .005
            assert abs(after_xyz.z-before_xyz.z) < .001
        drag_handle("z", (80, 145, 255), 0, -20)
        assert abs(commands[-1].setpoint_vect.z - xy.z) > .01
        def rpy():
            q = angular[-1].setpoint_quat
            return np.array([
                np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x*q.x + q.y*q.y)),
                np.arcsin(np.clip(2 * (q.w * q.y - q.z * q.x), -1., 1.)),
                np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z)),
            ])

        def drag_ring(axis, name):
            move(1120, 140)
            spin(.1)
            crop = screenshot(name + "-ring-before")[153:840, 342:1160].astype(np.int16)
            blue, green, red = crop.transpose(2, 0, 1)
            masks = [(red > green+35) & (red > blue+25),
                     (green > red+25) & (green > blue+15),
                     (blue > red+40) & (blue > green+40)]
            colors = [(235,75,75),(90,215,110),(80,145,255)]
            # Ring interiors are translucent; solid arrows and thin antialias
            # fringes are excluded. There are deliberately no grip dots.
            mask = masks[axis] & ~np.all(crop == np.array(colors[axis][::-1]), axis=2)
            mask = cv2.erode(mask.astype(np.uint8), np.ones((3,3),np.uint8))
            ys, xs = np.where(mask)
            assert len(xs), f"{name} ring not found"
            # A side of the ring gives a clear vertical drag tangent.
            index = np.argmin(xs) if axis != 2 else np.argmax(ys)
            before_angles, before_position = rpy(), commands[-1].setpoint_vect
            drag_at(xs[index]+342, ys[index]+153, 0 if axis != 2 else 16, -16 if axis != 2 else 0)
            change = rpy()-before_angles
            assert abs(change[axis]) > .005, f"{name} ring did not change its angle: {change}"
            change[axis]=0
            assert np.max(np.abs(change)) < .001, f"{name} changed another angle"
            assert commands[-1].setpoint_vect == before_position
            screenshot(name + "-ring-after")

        for axis, name in enumerate(["roll", "pitch", "yaw"]):
            drag_ring(axis, name)

        assert np.max(screenshot("follow-after-robot-drag")[106:124,617:637,1]) > 180, "Robot dragging disabled Follow"
        click(780,78)
        screenshot("top-panels-menu")
        click(1100,60)
        # Hiding the sidebar preserves active position control and the gizmo.
        before_hide = len(commands)
        mouse_drag(342, 250, -110, 0)
        hidden = screenshot("sidebar-hidden")
        assert len(commands) == before_hide
        red = np.all(hidden[64:94, 518:608] == np.array([48,41,166], np.uint8), axis=2)
        assert np.count_nonzero(red) > 100, "Sidebar did not hide / compact Kill is missing"
        drag_ring(2, "hidden-yaw")
        mouse_drag(26, 250, 80, 0)
        mouse_drag(326, 250, 16, 0)

        def key(name, pressed):
            code = d.keysym_to_keycode(XK.string_to_keysym(name))
            xtest.fake_input(d, X.KeyPress if pressed else X.KeyRelease, code)
            d.sync()

        # Editing drafts must not publish; only Command applies them.
        before_edit = len(commands)
        # Exact numeric RPY fields, including negative pitch, publish quaternions.
        for y, text in [(407, "12.0"), (440, "-8.0"), (473, "25.0")]:
            key("Control_L", True)
            click(280, y)
            key("a", True)
            key("a", False)
            spin(.08)
            key("Control_L", False)
            spin(.08)
            for character in text:
                name = {"-": "minus", ".": "period"}.get(character, character)
                key(name, True)
                key(name, False)
                spin(.06)
            key("Return", True)
            key("Return", False)
            spin(.2)
        assert len(commands) == before_edit, "Draft editing published a command"
        click(151, 520) # Explicit Command
        spin(.3)
        screenshot("numeric-rpy")
        assert np.max(np.abs(np.degrees(rpy()) - [12., -8., 25.])) < .01, f"Numeric RPY fields failed: {np.degrees(rpy())}"
        # Current copies telemetry without publishing.
        before_current = len(commands)
        click(65, 520)
        assert len(commands) == before_current
        # Dive uses actual XY/yaw, configured depth, and level roll/pitch.
        for pose in poses:
            pose.transform.translation.z = -1.5
        spin(.3)
        click(250, 520)
        spin(.3)
        assert abs(commands[-1].setpoint_vect.z + .75) < .001
        assert abs(commands[-1].setpoint_vect.x - 3.) < .001
        assert np.max(np.abs(rpy())) < .001
        screenshot("dive")
        # Modes are explicit; feedforward disables the pose gizmo.
        click(240, 249)
        spin(.3)
        screenshot("feedforward")
        assert commands[-1].mode == angular[-1].mode == ControllerCommand.FEEDFORWARD
        click(150, 249)
        spin(.3)
        assert commands[-1].mode == ControllerCommand.POSITION
        # Select/start a tree and observe its stack, then stop it.
        click(150, 789)
        tree_popup=screenshot("tree-options")
        spin(.7)
        tree_settled=screenshot("tree-options-settled")
        # The one-tree menu remains compact instead of growing every frame.
        for image in (tree_popup,tree_settled):
            assert np.array_equal(image[818,235],image[818,300]), "Tree popup exceeds its label width"
        click(140, 865)
        click(135, 827)
        spin(.5)
        screenshot("tree-running")
        assert mission["starts"] == 1
        before_auto = len(commands)
        click(151, 520)  # Manual command is blocked during autonomy.
        assert len(commands) == before_auto
        click(193, 827)
        spin(.5)
        assert mission["stops"] == 1
        screenshot("tree-stopped")
        click(135, 827)
        spin(.4)
        assert mission["starts"] == 2
        # Opening scene controls leaves the composition usable.
        click(406, 78)
        screenshot("scene-settings")
        key("Escape", True)
        key("Escape", False)
        # Verify a narrower window retains the sidebar and camera aspect ratio.
        window.configure(width=1280, height=800)
        d.sync()
        spin(.4)
        screenshot("narrow")
        click(100, 95) # Kill
        spin(.3)
        assert mission["stops"] == 2, "Kill did not stop the linked tree"
        assert kills[-1].switch_asserting_kill
        assert commands[-1].mode == angular[-1].mode == ControllerCommand.DISABLED
        screenshot("killed")
        before_compact = len(commands)
        mouse_drag(342, 250, -110, 0)
        click(560, 78) # The same compact toggle enables, then kills.
        assert kills and not kills[-1].switch_asserting_kill
        click(560, 78)
        assert kills[-1].switch_asserting_kill
        assert len(commands) > before_compact and commands[-1].mode == ControllerCommand.DISABLED
        screenshot("hidden-killed")
        print("PASS: XYZ/RPY dragging, Follow preservation, numeric edits, Current/Command/Dive, modes, pool Panels menu, sidebar hide/restore, tree start/stop, Kill and compact Enable/Kill; captures:", output)
    finally:
        for button in (1, 2, 3):
            xtest.fake_input(d, X.ButtonRelease, button)
        d.sync()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=15)
        log.close()
        action_server.destroy()
        node.destroy_node()
        rclpy.shutdown()
        d.close()


if __name__ == "__main__":
    if any(os.environ.get(phase) for phase in ("PANEL_LAYOUT_ONLY", "PANEL_CONTROLS_ONLY", "PANEL_CAPTURE_ONLY", "PANEL_TOOLS_ONLY", "PANEL_RVIZ_ONLY", "PANEL_VIEW_TOOLS_ONLY")):
        main()
    else:
        # Keep camera/layout and robot-control gestures isolated. Each phase
        # starts from the same viewport and has its own mocked ROS lifecycle.
        for phase in ("PANEL_LAYOUT_ONLY", "PANEL_CONTROLS_ONLY"):
            subprocess.run([sys.executable, str(Path(__file__).resolve())],
                           env={**os.environ, phase: "1"}, check=True)
