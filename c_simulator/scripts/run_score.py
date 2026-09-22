"""Compatibility imports for the 2026 score reference tests."""

from riptide_sim_config.profiles import load_behavior, share

_module = str(share("c_simulator") / "tasks/2026/behavior/scoring.py")
RunScore = load_behavior(_module + ":RunScore")
CourseJudge = load_behavior(_module + ":CourseJudge")
rotation_delta = load_behavior(_module + ":rotation_delta")
