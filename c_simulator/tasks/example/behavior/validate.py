"""Validate this pack before any simulator process starts."""

import math


def validate(config):
    regions = config.get("regions", [])
    ids = [r["id"] for r in regions]
    if not regions or len(ids) != len(set(ids)):
        raise ValueError("Observation regions require unique IDs and at least one region")
    for region in regions:
        if len(region["position"]) != 3 or not all(math.isfinite(v) for v in region["position"]):
            raise ValueError("Observation region position must be finite xyz")
        if any(not math.isfinite(region[k]) or region[k] <= 0 for k in ("radius", "dwell")):
            raise ValueError("Observation radius and dwell must be positive")
    for value in config["scoring_rules"].values():
        if not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError("Observation scoring values must be finite")
