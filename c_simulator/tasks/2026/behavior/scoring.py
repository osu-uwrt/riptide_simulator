"""RoboSub 2026 run ledger and deterministic geometry judge (no ROS).

Official sources and explicit simulation tolerances: docs/SCORING.md.
Referee-only awards are entered as a separate adjustment, never inferred.
"""

import math
from pathlib import Path
import yaml
import numpy as np


ROLES = {"repair": "fire", "rescue": "blood"}
LABELS = {
    "gate": "Gate passage / heading / role / style",
    "slalom_front": "Slalom row 1",
    "slalom_middle": "Slalom row 2",
    "slalom_back": "Slalom row 3",
    "bins": "Bin markers",
    "lights": "Magnet lights",
    "torpedoes": "Torpedo openings",
    "sequence": "Torpedo sequence bonus",
    "distance": "Torpedo distance bonuses",
    "surface": "Surface inside octagon",
    "facing": "Surface facing image",
    "objects_surface": "Surface with objects",
    "objects_drop": "Release objects",
    "baskets": "Place objects in baskets",
    "basket_count": "Signal basket count with turns",
    "home": "Return through gate",
    "pinger_first": "Random pinger: first task",
    "pinger_second": "Random pinger: second task",
}


class RunScore:
    def __init__(self, config=None):
        defaults = yaml.safe_load((Path(__file__).parents[1] / "config/scoring.yaml").read_text())
        self.rules = dict(defaults["points"], **(config or {}).get("points", {}))
        self.reset()

    def reset(self):
        self.running = False
        self.started = self.elapsed = 0.0
        self.role = None
        self.intended_role = "repair"
        self.random_role = self.heading_coin = False
        self.gate_passed = False
        self.gate_side = 0
        self.ended_reason = ""
        self.points = dict.fromkeys(LABELS, 0)
        self.adjustment = 0.0
        self.shots = {"torpedo": {}, "dropper": {}}
        self.lights = set()
        self.grasped = set()
        self.surfaced = set()
        self.dropped = set()
        self.basket_awards = {}
        self.basket_contents = {}
        self.pinger = {"mode": "disabled", "first": None, "active": None, "stage": 0}

    @property
    def accepts(self):
        return self.running and not self.ended_reason

    @property
    def eligible(self):
        return self.accepts and self.gate_passed

    @property
    def target_class(self):
        return ROLES[self.role or self.intended_role]

    def start(self, now, role="repair", heading_coin=False, role_coin=False):
        if role not in ROLES:
            raise ValueError("Select the known role: repair or rescue")
        self.reset()
        self.running = True
        self.started = float(now)
        self.random_role = bool(role_coin)
        self.intended_role = role
        self.heading_coin = bool(heading_coin)

    def stop(self, now):
        if self.running:
            self.elapsed = max(0.0, float(now) - self.started)
            self.running = False

    def award(self, key, value, task=None):
        if not self.accepts or value <= self.points[key]:
            return
        self.points[key] = int(value)
        p = self.pinger
        if task and p["mode"] == "random" and p["active"] == task:
            if p["stage"] == 0:
                self.points["pinger_first"] = self.rules["pinger_first"]
                p["stage"] = 1
            elif p["stage"] == 2 and task != p["first"]:
                self.points["pinger_second"] = self.rules["pinger_second"]
                p["stage"] = 3

    def gate(self, role, side, style=0):
        if not self.accepts:
            return
        if not self.gate_passed:
            self.role, self.gate_side = role, side
            self.gate_passed = True
        bonus = self.rules["role"] if self.random_role and self.role == self.intended_role else 0
        self.award(
            "gate", self.rules["gate"] + self.rules["heading"] * self.heading_coin + bonus + style
        )

    def slalom(self, key, side, depth):
        if self.eligible:
            self.award(
                key,
                (
                    self.rules["slalom_same"]
                    if side == self.gate_side
                    else self.rules["slalom_other"]
                )
                + self.rules["slalom_depth"] * bool(depth),
            )

    def release_payload(self, kind, ident, distance=0.0):
        shots = self.shots[kind]
        if self.accepts and ident not in shots and len(shots) < self.rules["max_shots"]:
            shots[ident] = dict(
                eligible=self.eligible, result=None, target="", distance=distance, correct=False
            )

    def payload_result(self, kind, ident, result, target, target_class, size=""):
        shot = self.shots[kind].get(ident)
        if not self.eligible or shot is None or not shot["eligible"] or shot["result"] is not None:
            return
        shot.update(
            result=result, target=target, correct=target_class == self.target_class, size=size
        )
        good = [s for s in self.shots[kind].values() if s["result"] in ("success", "wrong_target")]
        if kind == "dropper":
            unique = {s["target"] for s in good if s["correct"]}
            self.award(
                "bins", self.rules["bin"] * len(good) + self.rules["bin_class"] * len(unique)
            )
        else:
            self.award("torpedoes", self.rules["torpedo"] * len(good), "deploy")
            self.award(
                "distance",
                sum(
                    (
                        self.rules["distance_far"]
                        if s["distance"] >= self.rules["distance_far_m"]
                        else (
                            self.rules["distance_near"]
                            if s["distance"] >= self.rules["distance_near_m"]
                            else 0
                        )
                    )
                    for s in good
                ),
                "deploy",
            )
            shots = list(self.shots[kind].values())  # launch order, not arrival order
            if len(shots) == 2 and all(
                s["result"] in ("success", "wrong_target") and s["correct"] for s in shots
            ):
                if [s["size"] for s in shots] == ["large", "small"]:
                    self.award("sequence", self.rules["sequence"], "deploy")

    def light(self, key):
        if self.eligible:
            self.lights.add(key)
            self.award(
                "lights", self.rules["light"] * min(self.rules["max_lights"], len(self.lights))
            )

    def object_event(self, key, result, destination, correct_basket, object_role):
        if not self.eligible:
            return
        if result == "grasped":
            self.grasped.add(key)
            self.basket_contents.pop(key, None)
        elif result == "released" and key in self.grasped:
            self.dropped.add(key)
            self.award("objects_drop", self.rules["object_drop"] * len(self.dropped), "restore")
        elif result in ("success", "wrong_target") and destination in ("helmet", "warning"):
            self.basket_contents[key] = destination
            # Correct sorting earns the higher tier regardless of the robot's role.
            value = (
                self.rules["basket_correct"]
                if destination == correct_basket
                else self.rules["basket_other"]
            )
            self.basket_awards[key] = max(value, self.basket_awards.get(key, 0))
            self.award("baskets", sum(self.basket_awards.values()), "restore")

    def surface(self, facing=None, held=None):
        if not self.eligible:
            return
        self.award("surface", self.rules["surface"], "restore")
        if held is not None and held in self.grasped:
            self.surfaced.add(held)
            self.award(
                "objects_surface", self.rules["object_surface"] * len(self.surfaced), "restore"
            )
        icons = ("compass", "hammer_and_wrench") if self.role == "repair" else ("buoy", "sos")
        count = len(self.basket_contents)  # both baskets, per team clarification
        if facing is not None:
            value = self.rules["facing_correct"] if facing in icons else self.rules["facing_other"]
            if count > 0 and facing == icons[min(count, 2) - 1]:
                value = self.rules["facing_count"]
            self.award("facing", value, "restore")

    def basket_turns(self, turns):
        count = len(self.basket_contents)
        if self.eligible and count > 0 and turns > 0:
            value = (
                self.rules["basket_count"]
                if turns == count
                else self.rules["basket_count_near"] if abs(turns - count) == 1 else 0
            )
            self.award("basket_count", value, "restore")

    def select_pinger(self, task, randomized=False):
        if task not in ("deploy", "restore"):
            raise ValueError("Pinger task must be deploy or restore")
        if not self.accepts or self.pinger["mode"] != "disabled":
            raise ValueError("Select the pinger once at the start of an active run")
        if any(self.points[k] for k in ("torpedoes", "surface", "baskets", "objects_drop")):
            raise ValueError("Select the pinger before scoring a pinger task")
        self.pinger = dict(
            mode="random" if randomized else "specific", first=task, active=task, stage=0
        )

    def switch_pinger(self):
        p = self.pinger
        if not self.accepts or p["mode"] == "disabled":
            raise ValueError("No active pinger scoring session")
        if p["mode"] == "random" and p["stage"] == 0:
            p["mode"] = "specific"
        elif p["mode"] == "random" and p["stage"] == 1:
            p["stage"] = 2
        p["active"] = "restore" if p["active"] == "deploy" else "deploy"

    def snapshot(self, now):
        elapsed = max(0.0, float(now) - self.started) if self.running else self.elapsed
        return dict(
            running=self.running,
            elapsed=elapsed,
            scoring_open=self.accepts,
            ended_reason=self.ended_reason,
            intended_role=self.intended_role,
            role=self.role,
            target_class=self.target_class,
            gate_passed=self.gate_passed,
            total=sum(self.points.values()) + self.adjustment,
            adjustment=self.adjustment,
            rows=[dict(key=k, label=LABELS[k], points=v) for k, v in self.points.items()],
            basket_count=len(self.basket_contents),
            pinger=dict(self.pinger),
            time_bonus_eligible=bool(
                self.points["surface"]
                and any(self.points[k] for k in ("slalom_front", "slalom_middle", "slalom_back"))
                and (self.points["bins"] or self.points["torpedoes"])
            ),
        )


def rotation_delta(a, b):
    """Small successive rotation in body axes, safe across Euler wrap/gimbal lock."""
    r = a.T @ b
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / 2
    angle = math.atan2(np.linalg.norm(v), np.clip((np.trace(r) - 1) / 2, -1, 1))
    return v * (angle / np.linalg.norm(v)) if np.linalg.norm(v) > 1e-9 else v


class CourseJudge:
    def __init__(self, score, cfg, frames, vertices):
        self.score, self.cfg, self.frames = score, cfg, frames
        self.vertices = np.asarray(vertices)
        self.turn_log = []  # drained by the node; survives judge resets
        self.reset()

    def reset(self):
        self.previous = None
        self.gate_attempt = None
        self.gate_entry_side = None
        self.submerged = False
        self.surface_dwell = 0.0
        self.facing_dwell = 0.0
        self.facing = None
        self.previous_count = 0
        self.reset_turns()
        self.gate_crossing = None

    def reset_turns(self):
        self.restore_yaw = self.settle_yaw = self.spin_start = self.spin_peak = 0.0
        self.turn_dwell = 0.0
        self.turn_judged = False
        self.turn_tracking = False

    def judge_turns(self, reason):
        """Judge the one-direction rotation so far (start to furthest heading).
        Stops keep it going; only a reversal ends it (see update)."""
        travel = abs(self.spin_peak - self.spin_start)
        if travel < math.radians(30):
            return
        tolerance = math.radians(self.cfg["turn_tolerance_deg"])
        turns = int((travel + tolerance) // (2 * math.pi))
        before = self.score.points["basket_count"]
        self.score.basket_turns(turns)
        after = self.score.points["basket_count"]
        self.log_turn(
            f"{reason}: rotation {math.degrees(travel):.0f} deg -> {turns} "
            f"turn(s) for {len(self.score.basket_contents)} object(s); basket_count "
            + (f"{before} -> {after}" if after != before else f"stays {after}")
        )

    def log_turn(self, message):
        self.turn_log.append("Basket turns: " + message)

    def local(self, key, body):
        return np.linalg.inv(self.frames[key]) @ body

    def update(self, body, dt, held=None):
        s, c = self.score, self.cfg
        if not s.accepts:
            self.previous = None
            return
        if (
            self.previous is not None
            and np.linalg.norm(body[:3, 3] - self.previous[:3, 3]) > c["max_pose_step"]
        ):
            self.finish_gate_attempt()
            self.reset()
        world = self.vertices @ body[:3, :3].T + body[:3, 3]
        top = float(world[:, 2].max())
        if top < c["surface_z"] - c["breach_margin"]:
            self.submerged = True
        gate = self.local("gate", body)
        gate_near = np.linalg.norm(gate[:3, 3]) <= c["attempt_radius"]
        if gate_near and self.gate_attempt is None:
            self.gate_attempt = dict(turns=np.zeros(3), passed=False, role=None, side=0)
        if self.previous is not None:
            old_gate = self.local("gate", self.previous)
            if (old_gate[0, 3] > 0 >= gate[0, 3]) or (old_gate[0, 3] < 0 <= gate[0, 3]):
                f = old_gate[0, 3] / (old_gate[0, 3] - gate[0, 3])
                cross = old_gate[:3, 3] + f * (gate[:3, 3] - old_gate[:3, 3])
                self.gate_crossing = cross
            # Record a full vehicle traversal, not an oscillation on the plane.
            gv = self.vertices @ gate[:3, :3].T + gate[:3, 3]
            side = 1 if gv[:, 0].min() > 0 else -1 if gv[:, 0].max() < 0 else 0
            if side and self.gate_entry_side is not None and side != self.gate_entry_side:
                if self.gate_crossing is not None:
                    y = self.gate_crossing[1]
                    at_plane = self.vertices @ gate[:3, :3].T + self.gate_crossing
                    fits = (
                        np.max(np.abs(at_plane[:, 1])) < c["gate_half_width"]
                        and at_plane[:, 2].max() < c["gate_top"]
                        and np.max(np.abs(gv[:, 1])) < c["gate_half_width"]
                        and gv[:, 2].max() < c["gate_top"]
                        and world[:, 2].min() > c.get("floor_z", -2.1336)
                    )
                    if fits and self.gate_entry_side == 1 and self.gate_attempt is not None:
                        repair_y = (
                            np.linalg.inv(self.frames["gate"]) @ self.frames["gate_repair"]
                        )[1, 3]
                        role = "repair" if y * repair_y > 0 else "rescue"
                        self.gate_attempt.update(passed=True, role=role, side=1 if y > 0 else -1)
                        s.gate(role, self.gate_attempt["side"])
                    elif (
                        fits and self.gate_entry_side == -1 and s.eligible and top < c["surface_z"]
                    ):
                        s.award("home", s.rules["home"])
                self.gate_crossing = None
            if side:
                self.gate_entry_side = side
            if gate_near and self.gate_attempt is not None:
                self.gate_attempt["turns"] += rotation_delta(self.previous[:3, :3], body[:3, :3])
            if s.eligible:
                for key in ("slalom_front", "slalom_middle", "slalom_back"):
                    a, b = self.local(key, self.previous), self.local(key, body)
                    if a[0, 3] > 0 >= b[0, 3]:
                        f = a[0, 3] / (a[0, 3] - b[0, 3])
                        p = a[:3, 3] + f * (b[:3, 3] - a[:3, 3])
                        if 0 < abs(p[1]) < c["slalom_half_width"]:
                            ext = np.abs(b[:3, :3]) @ np.max(np.abs(self.vertices), axis=0)
                            depth = (
                                p[2] + ext[2] >= c["slalom_bottom"]
                                and p[2] - ext[2] <= c["slalom_top"]
                            )
                            s.slalom(key, 1 if p[1] > 0 else -1, depth)
        else:
            self.gate_entry_side = 1 if gate[0, 3] > 0 else -1
        if not gate_near:
            self.finish_gate_attempt()
        octagon = self.local("octagon", body)
        vertices = self.vertices @ octagon[:3, :3].T + octagon[:3, 3]
        normals = np.c_[np.cos(np.arange(8) * math.pi / 4), np.sin(np.arange(8) * math.pi / 4)]
        inside = bool(np.all(vertices[:, :2] @ normals.T <= c["octagon_apothem"]))
        if self.submerged and top > c["surface_z"] + c["breach_margin"] and not inside:
            self.finish_gate_attempt()
            s.ended_reason = "Breach outside octagon; scoring ended (timer remains manual)"
        at_surface = self.submerged and inside and top >= c["surface_z"]
        restore_near = np.linalg.norm(self.local("table", body)[:3, 3]) <= c["attempt_radius"]
        if restore_near and s.eligible:
            count = len(s.basket_contents)
            if not self.turn_tracking:
                self.reset_turns()
                self.previous_count = count
                self.log_turn(f"entered table area with {count} object(s) in baskets")
            elif count != self.previous_count:
                self.log_turn(
                    f"basket count {self.previous_count} -> {count}; discarding "
                    f"{math.degrees(abs(self.spin_peak - self.spin_start)):.0f} deg of rotation"
                )
                self.reset_turns()
                self.previous_count = count
            elif self.previous is not None:
                yaw = math.atan2(body[1, 0], body[0, 0])
                last = math.atan2(self.previous[1, 0], self.previous[0, 0])
                self.restore_yaw += math.atan2(math.sin(yaw - last), math.cos(yaw - last))
                if abs(self.restore_yaw - self.spin_start) > abs(self.spin_peak - self.spin_start):
                    self.spin_peak = self.restore_yaw
                # Backing off the furthest heading by more than turn_reversal_deg
                # ends the rotation at that peak; the count restarts from there.
                elif abs(self.spin_peak - self.restore_yaw) > math.radians(c["turn_reversal_deg"]):
                    self.judge_turns("rotation reversed")
                    self.log_turn(
                        f"direction reversed after {math.degrees(abs(self.spin_peak - self.spin_start)):.0f}"
                        " deg of rotation; count restarts"
                    )
                    self.spin_start, self.spin_peak = self.spin_peak, self.restore_yaw
                # Stopped = heading held within turn_settle_deg for turn_dwell (a
                # time window, so stale ground-truth ticks never read as a stop).
                if abs(self.restore_yaw - self.settle_yaw) <= math.radians(c["turn_settle_deg"]):
                    self.turn_dwell += dt
                else:
                    self.settle_yaw, self.turn_dwell = self.restore_yaw, 0.0
                    self.turn_judged = False
            self.turn_tracking = True
            if self.turn_dwell >= c["turn_dwell"] and not self.turn_judged:
                self.turn_judged = True
                self.judge_turns("rotation stopped")  # judged, but the count continues
        elif self.turn_tracking:
            if s.eligible:
                self.judge_turns("left table area")
            self.reset_turns()
        if at_surface and s.eligible:
            self.surface_dwell += dt
            if self.surface_dwell >= c["surface_dwell"]:
                s.surface(held=held)
                candidates = []
                heading = body[:2, 0]
                for key in ("compass", "hammer_and_wrench", "buoy", "sos"):
                    direction = self.frames[key][:2, 3] - body[:2, 3]
                    denom = np.linalg.norm(direction) * np.linalg.norm(heading)
                    angle = (
                        math.acos(np.clip(np.dot(direction, heading) / denom, -1, 1))
                        if denom > 1e-9
                        else math.pi
                    )
                    candidates.append((angle, key))
                angle, facing = min(candidates)
                if angle <= math.radians(c["facing_tolerance_deg"]):
                    self.facing_dwell = self.facing_dwell + dt if facing == self.facing else 0.0
                    self.facing = facing
                    if self.facing_dwell >= c["facing_dwell"]:
                        s.surface(facing, held)
                else:
                    self.facing_dwell = 0.0
                    self.facing = None
        else:
            self.surface_dwell = self.facing_dwell = 0.0
            self.facing = None
        self.previous = body.copy()

    def finish_gate_attempt(self):
        a = self.gate_attempt
        if a is not None and a["passed"]:
            quarters = np.floor((np.abs(a["turns"]) + 1e-6) / (math.pi / 2)).astype(int)
            rp = min(self.score.rules["max_style_quarters"], int(quarters[0] + quarters[1]))
            yaw = min(self.score.rules["max_style_quarters"] - rp, int(quarters[2]))
            self.score.gate(
                a["role"],
                a["side"],
                self.score.rules["style_rp"] * rp + self.score.rules["style_yaw"] * yaw,
            )
        self.gate_attempt = None
