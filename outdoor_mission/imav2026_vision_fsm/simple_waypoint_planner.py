"""Deliberately simple, deterministic waypoint planning for IMAV 2026.

The competition-specific coordinates are supplied on the competition day.  This
module does not optimise online: it validates a pre-agreed route table and sends
waypoints in order.  A small lawnmower helper is provided for rectangular search
and mapping areas.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Tuple


EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GeoPoint:
    latitude_deg: float
    longitude_deg: float
    altitude_agl_m: float


@dataclass(frozen=True)
class Waypoint:
    name: str
    position: GeoPoint
    acceptance_radius_m: float = 3.0


@dataclass(frozen=True)
class MissionPlan:
    """Routes consumed by the macro FSM.

    Required route IDs:
      m3_water_source, m3_water_target, m1_area1, m2_fire_search,
      m4_deadman_search, home.
    m1_area2 is required only when include_mapping_area2 is true.
    """

    routes: Mapping[str, Tuple[Waypoint, ...]]
    takeoff_altitude_m: float = 30.0
    water_goal_l: float = 2.0
    first_aid_drop_altitude_m: float = 1.5
    first_aid_horizontal_goal_m: float = 0.5
    deadman_approach_altitude_m: float = 10.0
    include_mapping_area2: bool = True
    low_battery_return_percent: float = 25.0
    slot_duration_s: float = 1800.0
    return_time_margin_s: float = 180.0

    def validate(self) -> None:
        required = {
            "m3_water_source",
            "m3_water_target",
            "m1_area1",
            "m2_fire_search",
            "m4_deadman_search",
            "home",
        }
        if self.include_mapping_area2:
            required.add("m1_area2")
        missing = sorted(route_id for route_id in required if not self.routes.get(route_id))
        if missing:
            raise ValueError(f"Mission plan has missing or empty routes: {', '.join(missing)}")
        if not 0.0 < self.first_aid_drop_altitude_m <= 2.0:
            raise ValueError("first_aid_drop_altitude_m must be in (0, 2.0]")
        if not 2.0 < self.deadman_approach_altitude_m <= 80.0:
            raise ValueError("deadman_approach_altitude_m must be in (2.0, 80.0]")
        if not 0.0 < self.first_aid_horizontal_goal_m <= 3.0:
            raise ValueError("first_aid_horizontal_goal_m must be in (0, 3.0]")
        if self.water_goal_l <= 0.0 or self.water_goal_l > 2.0:
            raise ValueError("water_goal_l must be in (0, 2.0]")
        if self.slot_duration_s <= 0.0:
            raise ValueError("slot_duration_s must be positive")
        if not 0.0 < self.return_time_margin_s < self.slot_duration_s:
            raise ValueError("return_time_margin_s must be between zero and slot_duration_s")
        if not 0.0 < self.takeoff_altitude_m <= 80.0:
            raise ValueError("takeoff_altitude_m must be in (0, 80.0]")
        invalid_altitudes = [
            waypoint.name
            for route in self.routes.values()
            for waypoint in route
            if not 0.0 <= waypoint.position.altitude_agl_m <= 80.0
        ]
        if invalid_altitudes:
            raise ValueError(f"Waypoint altitude outside 0-80 m AGL: {', '.join(invalid_altitudes)}")

    def route(self, route_id: str) -> Tuple[Waypoint, ...]:
        return tuple(self.routes[route_id])


def _distance_m(a: GeoPoint, b: GeoPoint) -> float:
    lat1 = math.radians(a.latitude_deg)
    lat2 = math.radians(b.latitude_deg)
    dlat = lat2 - lat1
    dlon = math.radians(b.longitude_deg - a.longitude_deg)
    h = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def _interpolate(a: GeoPoint, b: GeoPoint, fraction: float, altitude_agl_m: float) -> GeoPoint:
    return GeoPoint(
        latitude_deg=a.latitude_deg + (b.latitude_deg - a.latitude_deg) * fraction,
        longitude_deg=a.longitude_deg + (b.longitude_deg - a.longitude_deg) * fraction,
        altitude_agl_m=altitude_agl_m,
    )


def generate_lawnmower_route(
    south_west: GeoPoint,
    north_west: GeoPoint,
    south_east: GeoPoint,
    north_east: GeoPoint,
    lane_spacing_m: float,
    altitude_agl_m: float,
    prefix: str,
) -> Tuple[Waypoint, ...]:
    """Generate alternating west/east endpoints for an approximately rectangular area.

    The perception/mapping team must choose lane spacing from camera footprint and
    required overlap.  Linear lat/lon interpolation is adequate at this field scale.
    """

    if lane_spacing_m <= 0.0:
        raise ValueError("lane_spacing_m must be positive")
    west_length = _distance_m(south_west, north_west)
    east_length = _distance_m(south_east, north_east)
    lane_count = max(2, math.ceil(max(west_length, east_length) / lane_spacing_m) + 1)
    waypoints = []
    for lane_index in range(lane_count):
        fraction = lane_index / (lane_count - 1)
        west = _interpolate(south_west, north_west, fraction, altitude_agl_m)
        east = _interpolate(south_east, north_east, fraction, altitude_agl_m)
        endpoints = (west, east) if lane_index % 2 == 0 else (east, west)
        for side, point in zip(("A", "B"), endpoints):
            waypoints.append(
                Waypoint(
                    name=f"{prefix}_lane_{lane_index + 1:02d}_{side}",
                    position=point,
                    acceptance_radius_m=5.0,
                )
            )
    return tuple(waypoints)


def load_plan_json(path: str | Path) -> MissionPlan:
    """Load the day-of-competition route table from JSON."""

    data = json.loads(Path(path).read_text(encoding="utf-8"))
    routes: Dict[str, Tuple[Waypoint, ...]] = {}
    for route_id, raw_waypoints in data["routes"].items():
        routes[route_id] = tuple(
            Waypoint(
                name=item["name"],
                position=GeoPoint(
                    latitude_deg=float(item["latitude_deg"]),
                    longitude_deg=float(item["longitude_deg"]),
                    altitude_agl_m=float(item["altitude_agl_m"]),
                ),
                acceptance_radius_m=float(item.get("acceptance_radius_m", 3.0)),
            )
            for item in raw_waypoints
        )
    plan = MissionPlan(
        routes=routes,
        takeoff_altitude_m=float(data.get("takeoff_altitude_m", 30.0)),
        water_goal_l=float(data.get("water_goal_l", 2.0)),
        first_aid_drop_altitude_m=float(data.get("first_aid_drop_altitude_m", 1.5)),
        first_aid_horizontal_goal_m=float(data.get("first_aid_horizontal_goal_m", 0.5)),
        deadman_approach_altitude_m=float(data.get("deadman_approach_altitude_m", 10.0)),
        include_mapping_area2=bool(data.get("include_mapping_area2", True)),
        low_battery_return_percent=float(data.get("low_battery_return_percent", 25.0)),
        slot_duration_s=float(data.get("slot_duration_s", 1800.0)),
        return_time_margin_s=float(data.get("return_time_margin_s", 180.0)),
    )
    plan.validate()
    return plan


def write_plan_json(plan: MissionPlan, path: str | Path) -> None:
    """Write a validated plan in a format that can be reviewed before flight."""

    plan.validate()
    payload = {
        "takeoff_altitude_m": plan.takeoff_altitude_m,
        "water_goal_l": plan.water_goal_l,
        "first_aid_drop_altitude_m": plan.first_aid_drop_altitude_m,
        "first_aid_horizontal_goal_m": plan.first_aid_horizontal_goal_m,
        "deadman_approach_altitude_m": plan.deadman_approach_altitude_m,
        "include_mapping_area2": plan.include_mapping_area2,
        "low_battery_return_percent": plan.low_battery_return_percent,
        "slot_duration_s": plan.slot_duration_s,
        "return_time_margin_s": plan.return_time_margin_s,
        "routes": {
            route_id: [
                {
                    "name": waypoint.name,
                    "latitude_deg": waypoint.position.latitude_deg,
                    "longitude_deg": waypoint.position.longitude_deg,
                    "altitude_agl_m": waypoint.position.altitude_agl_m,
                    "acceptance_radius_m": waypoint.acceptance_radius_m,
                }
                for waypoint in waypoints
            ]
            for route_id, waypoints in plan.routes.items()
        },
    }
    Path(path).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
