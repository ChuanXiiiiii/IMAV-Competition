"""Simple, deterministic waypoint plans for IMAV 2026 Outdoor Missions 1 and 4."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Dict, Mapping, Sequence, Tuple


EARTH_RADIUS_M = 6_371_000.0
SUPPORTED_MISSIONS = (1, 4)


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
    """Configuration shared by test profiles and the combined competition FSM."""

    routes: Mapping[str, Tuple[Waypoint, ...]]
    enabled_missions: Tuple[int, ...] = (1, 4)
    takeoff_altitude_m: float = 30.0
    include_mapping_area2: bool = True
    first_aid_drop_altitude_m: float = 1.5
    first_aid_horizontal_goal_m: float = 0.5
    deadman_approach_altitude_m: float = 10.0
    low_battery_return_percent: float = 25.0
    slot_duration_s: float = 1800.0
    return_time_margin_s: float = 180.0

    def with_missions(self, missions: Sequence[int]) -> "MissionPlan":
        """Return a plan for M1-only, M4-only, or combined execution."""

        return replace(self, enabled_missions=normalise_missions(missions))

    def validate(self) -> None:
        missions = normalise_missions(self.enabled_missions)
        required = {"home"}
        if 1 in missions:
            required.add("m1_area1")
            if self.include_mapping_area2:
                required.add("m1_area2")
        if 4 in missions:
            required.add("m4_deadman_search")

        missing = sorted(route_id for route_id in required if not self.routes.get(route_id))
        if missing:
            raise ValueError(f"Mission plan has missing or empty routes: {', '.join(missing)}")
        if not 0.0 < self.first_aid_drop_altitude_m < 2.0:
            raise ValueError("first_aid_drop_altitude_m must be strictly between 0 and 2.0 m")
        if not 2.0 < self.deadman_approach_altitude_m <= 80.0:
            raise ValueError("deadman_approach_altitude_m must be in (2.0, 80.0]")
        if not 0.0 < self.first_aid_horizontal_goal_m <= 3.0:
            raise ValueError("first_aid_horizontal_goal_m must be in (0, 3.0]")
        if not 0.0 < self.takeoff_altitude_m <= 80.0:
            raise ValueError("takeoff_altitude_m must be in (0, 80.0]")
        if self.slot_duration_s <= 0.0:
            raise ValueError("slot_duration_s must be positive")
        if not 0.0 < self.return_time_margin_s < self.slot_duration_s:
            raise ValueError("return_time_margin_s must be between zero and slot_duration_s")
        invalid_altitudes = [
            waypoint.name
            for route_id in required
            for waypoint in self.routes[route_id]
            if not 0.0 <= waypoint.position.altitude_agl_m <= 80.0
        ]
        if invalid_altitudes:
            raise ValueError(f"Waypoint altitude outside 0-80 m AGL: {', '.join(invalid_altitudes)}")

    def route(self, route_id: str) -> Tuple[Waypoint, ...]:
        return tuple(self.routes[route_id])


def normalise_missions(missions: Sequence[int]) -> Tuple[int, ...]:
    result = tuple(dict.fromkeys(int(mission) for mission in missions))
    if not result:
        raise ValueError("At least one mission must be enabled")
    unsupported = sorted(set(result) - set(SUPPORTED_MISSIONS))
    if unsupported:
        raise ValueError(f"Only Outdoor Missions 1 and 4 are supported: {unsupported}")
    return result


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
    """Generate a predictable alternating route for M1 mapping or M4 search."""

    if lane_spacing_m <= 0.0:
        raise ValueError("lane_spacing_m must be positive")
    if not 0.0 < altitude_agl_m <= 80.0:
        raise ValueError("altitude_agl_m must be in (0, 80.0]")
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


def load_plan_json(path: str | Path, mission_override: Sequence[int] | None = None) -> MissionPlan:
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
    configured = mission_override if mission_override is not None else data.get("enabled_missions", [1, 4])
    plan = MissionPlan(
        routes=routes,
        enabled_missions=normalise_missions(configured),
        takeoff_altitude_m=float(data.get("takeoff_altitude_m", 30.0)),
        include_mapping_area2=bool(data.get("include_mapping_area2", True)),
        first_aid_drop_altitude_m=float(data.get("first_aid_drop_altitude_m", 1.5)),
        first_aid_horizontal_goal_m=float(data.get("first_aid_horizontal_goal_m", 0.5)),
        deadman_approach_altitude_m=float(data.get("deadman_approach_altitude_m", 10.0)),
        low_battery_return_percent=float(data.get("low_battery_return_percent", 25.0)),
        slot_duration_s=float(data.get("slot_duration_s", 1800.0)),
        return_time_margin_s=float(data.get("return_time_margin_s", 180.0)),
    )
    plan.validate()
    return plan
