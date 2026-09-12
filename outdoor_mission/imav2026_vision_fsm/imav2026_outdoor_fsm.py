"""Pluggable test and competition FSM for IMAV 2026 Outdoor Missions 1 and 4.

Run profiles are selected through MissionPlan.enabled_missions or the CLI
--missions override.  The same state machine can therefore exercise Mission 1,
Mission 4, or the combined Mission 1 -> Mission 4 competition sequence.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, Optional, Tuple

from simple_waypoint_planner import GeoPoint, MissionPlan, Waypoint, load_plan_json


class MissionState(str, Enum):
    PRE_FLIGHT = "PRE_FLIGHT"
    TAKEOFF = "TAKEOFF"
    M1_SURVEY_AREA_1 = "M1_SURVEY_AREA_1"
    M1_SURVEY_AREA_2 = "M1_SURVEY_AREA_2"
    M4_SEARCH = "M4_SEARCH"
    M4_APPROACH = "M4_APPROACH"
    M4_DESCEND = "M4_DESCEND"
    M4_RELEASE = "M4_RELEASE"
    RETURN_HOME = "RETURN_HOME"
    PRECISION_LANDING = "PRECISION_LANDING"
    POST_FLIGHT_EXPORT = "POST_FLIGHT_EXPORT"
    COMPLETE = "COMPLETE"
    EMERGENCY_LAND = "EMERGENCY_LAND"


class FlightCommand(str, Enum):
    HOLD = "HOLD"
    TAKEOFF = "TAKEOFF"
    FOLLOW_ROUTE = "FOLLOW_ROUTE"
    HOLD_POSITION = "HOLD_POSITION"
    DESCEND_TO_ALTITUDE = "DESCEND_TO_ALTITUDE"
    PRECISION_LAND = "PRECISION_LAND"
    EMERGENCY_LAND = "EMERGENCY_LAND"
    DISARMED = "DISARMED"


class VisionMode(str, Enum):
    IDLE = "IDLE"
    MAP_AND_IDENTIFY_VEHICLES = "MAP_AND_IDENTIFY_VEHICLES"
    MANNEQUIN_AND_LED_SEARCH = "MANNEQUIN_AND_LED_SEARCH"
    FIRST_AID_DROP_ALIGNMENT = "FIRST_AID_DROP_ALIGNMENT"
    ARUCO_ID0_PRECISION_LANDING = "ARUCO_ID0_PRECISION_LANDING"


class SoundMode(str, Enum):
    OFF = "OFF"
    DEADMAN_2_6_TO_3_0_KHZ = "DEADMAN_2_6_TO_3_0_KHZ"


class MechanismCommand(str, Enum):
    SAFE = "SAFE"
    ARM_FIRST_AID = "ARM_FIRST_AID"
    RELEASE_FIRST_AID = "RELEASE_FIRST_AID"


@dataclass(frozen=True)
class MissionPlugin:
    mission_id: int
    name: str
    first_state: MissionState
    required_routes: Tuple[str, ...]


MISSION_PLUGINS = {
    1: MissionPlugin(1, "Mapping and vehicle identification", MissionState.M1_SURVEY_AREA_1, ("m1_area1",)),
    4: MissionPlugin(4, "First-aid drop", MissionState.M4_SEARCH, ("m4_deadman_search",)),
}


@dataclass(frozen=True)
class LocatedObservation:
    observation_id: str
    label: str
    position: GeoPoint
    confidence: float
    position_error_m: float


@dataclass(frozen=True)
class OperatorInput:
    start: bool = False
    abort: bool = False
    force_return: bool = False
    post_flight_export_complete: bool = False


@dataclass(frozen=True)
class AutopilotInput:
    preflight_ok: bool = False
    airborne: bool = False
    landed: bool = True
    gps_ok: bool = False
    inside_geofence: bool = True
    battery_percent: float = 100.0
    altitude_agl_m: float = 0.0
    completed_route_id: Optional[str] = None
    airframe_type: str = "multicopter"


@dataclass(frozen=True)
class VisionInput:
    detection_ready: bool = False
    localisation_and_calibration_ready: bool = False
    vehicle_fixes: Tuple[LocatedObservation, ...] = ()
    deadman_led_detected: bool = False
    deadman_target_fix: Optional[GeoPoint] = None
    mannequin_target_fix: Optional[GeoPoint] = None
    drop_horizontal_error_m: Optional[float] = None
    landing_marker_visible: bool = False


@dataclass(frozen=True)
class MappingInput:
    ready: bool = False
    area_1_coverage_complete: bool = False
    area_2_coverage_complete: bool = False
    map_export_ready: bool = False
    map_artifact_uri: Optional[str] = None


@dataclass(frozen=True)
class SoundInput:
    ready: bool = False
    deadman_detected: bool = False
    frequency_hz: Optional[float] = None
    confidence: float = 0.0
    target_fix: Optional[GeoPoint] = None
    target_position_error_m: Optional[float] = None


@dataclass(frozen=True)
class MechanismInput:
    ready: bool = False
    fault: bool = False
    first_aid_loaded: bool = False
    first_aid_kit_verified: bool = False
    first_aid_armed: bool = False
    first_aid_release_complete: bool = False


@dataclass(frozen=True)
class FSMInput:
    now_s: float
    operator: OperatorInput = field(default_factory=OperatorInput)
    autopilot: AutopilotInput = field(default_factory=AutopilotInput)
    vision: VisionInput = field(default_factory=VisionInput)
    mapping: MappingInput = field(default_factory=MappingInput)
    sound: SoundInput = field(default_factory=SoundInput)
    mechanism: MechanismInput = field(default_factory=MechanismInput)


@dataclass(frozen=True)
class FSMOutput:
    state: MissionState
    profile_missions: Tuple[int, ...]
    active_mission: Optional[int]
    flight_command: FlightCommand
    route_id: Optional[str] = None
    route: Tuple[Waypoint, ...] = ()
    route_repeat: bool = False
    target_altitude_agl_m: Optional[float] = None
    vision_mode: VisionMode = VisionMode.IDLE
    sound_mode: SoundMode = SoundMode.OFF
    mechanism_command: MechanismCommand = MechanismCommand.SAFE
    export_results: bool = False
    accomplished_missions: Tuple[int, ...] = ()
    validated_missions: Tuple[int, ...] = ()
    accepted_vehicle_count: int = 0
    report_deadline_remaining_s: Optional[float] = None
    slot_deadline_remaining_s: Optional[float] = None
    status: str = ""


class OutdoorMissionFSM:
    """Runtime-selectable orchestrator for M1-only, M4-only, or M1+M4.

    Mission plugins are executed in the order listed in enabled_missions.  A
    plugin marks its task action accomplished; V4-1 validation occurs only after
    the aircraft safely returns to the designated landing area.
    """

    def __init__(self, plan: MissionPlan) -> None:
        plan.validate()
        self.plan = plan
        self.profile = tuple(MISSION_PLUGINS[mission] for mission in plan.enabled_missions)
        self.plugin_index = 0
        self.state = MissionState.PRE_FLIGHT
        self.accomplished_missions: set[int] = set()
        self.validated_missions: set[int] = set()
        self.vehicle_records: Dict[str, LocatedObservation] = {}
        self.m4_target: Optional[GeoPoint] = None
        self.m4_target_is_deadman = False
        self.first_aid_release_altitude_m: Optional[float] = None
        self.first_aid_horizontal_error_m: Optional[float] = None
        self.started_at_s: Optional[float] = None
        self.landed_at_s: Optional[float] = None
        self.latest_now_s = 0.0
        self.latest_map_artifact_uri: Optional[str] = None
        self.last_transition_reason = f"Ready for profile {plan.enabled_missions}."

    @property
    def enabled_missions(self) -> Tuple[int, ...]:
        return self.plan.enabled_missions

    def transition_to(self, state: MissionState, reason: str) -> None:
        if state != self.state:
            print(f"[FSM] {self.state.value} -> {state.value}: {reason}")
            self.state = state
        self.last_transition_reason = reason

    def _preflight_missing(self, inputs: FSMInput) -> Tuple[str, ...]:
        missing = []
        if not inputs.autopilot.preflight_ok or not inputs.autopilot.gps_ok:
            missing.append("autopilot/GPS")
        if inputs.autopilot.battery_percent <= self.plan.low_battery_return_percent:
            missing.append("battery above return reserve")
        if not inputs.vision.detection_ready or not inputs.vision.localisation_and_calibration_ready:
            missing.append("vision/localisation")
        if 1 in self.enabled_missions and not inputs.mapping.ready:
            missing.append("mapping")
        if 4 in self.enabled_missions:
            if not inputs.sound.ready:
                missing.append("sound localisation")
            if (
                not inputs.mechanism.ready
                or inputs.mechanism.fault
                or not inputs.mechanism.first_aid_loaded
                or not inputs.mechanism.first_aid_kit_verified
            ):
                missing.append("verified V4-1 first-aid mechanism/payload")
        return tuple(missing)

    def _route_complete(self, inputs: FSMInput, route_id: str) -> bool:
        return inputs.autopilot.completed_route_id == route_id

    def _capture_vehicle_records(self, inputs: FSMInput) -> None:
        if self.state not in (MissionState.M1_SURVEY_AREA_1, MissionState.M1_SURVEY_AREA_2):
            return
        for observation in inputs.vision.vehicle_fixes:
            if observation.confidence >= 0.5 and observation.position_error_m <= 5.0 and observation.label.strip():
                self.vehicle_records[observation.observation_id] = observation

    def _start_current_plugin(self) -> None:
        plugin = self.profile[self.plugin_index]
        self.transition_to(plugin.first_state, f"Starting Mission {plugin.mission_id}: {plugin.name}.")

    def _finish_plugin(self, mission_id: int, reason: str) -> None:
        self.accomplished_missions.add(mission_id)
        self.plugin_index += 1
        if self.plugin_index < len(self.profile):
            next_plugin = self.profile[self.plugin_index]
            self.transition_to(next_plugin.first_state, f"{reason} Starting Mission {next_plugin.mission_id}.")
        else:
            self.transition_to(MissionState.RETURN_HOME, f"{reason} All selected task actions are complete.")

    def step(self, inputs: FSMInput) -> FSMOutput:
        self.latest_now_s = inputs.now_s
        if inputs.mapping.map_artifact_uri:
            self.latest_map_artifact_uri = inputs.mapping.map_artifact_uri
        self._capture_vehicle_records(inputs)

        if self.state not in (MissionState.COMPLETE, MissionState.EMERGENCY_LAND):
            if inputs.operator.abort or not inputs.autopilot.inside_geofence:
                self.transition_to(MissionState.EMERGENCY_LAND, "Abort or geofence violation.")
            elif (
                inputs.operator.force_return
                or inputs.autopilot.battery_percent <= self.plan.low_battery_return_percent
                or (4 in self.enabled_missions and inputs.mechanism.fault)
                or self._return_margin_reached(inputs.now_s)
            ) and self.state not in (
                MissionState.PRE_FLIGHT,
                MissionState.RETURN_HOME,
                MissionState.PRECISION_LANDING,
                MissionState.POST_FLIGHT_EXPORT,
            ):
                self.transition_to(MissionState.RETURN_HOME, "Safe return requested by reserve/fault/time guard.")

        if self.state == MissionState.PRE_FLIGHT:
            missing = self._preflight_missing(inputs)
            if inputs.operator.start and not missing:
                self.started_at_s = inputs.now_s
                self.transition_to(MissionState.TAKEOFF, "All enabled-plugin readiness checks passed.")
            elif inputs.operator.start:
                self.last_transition_reason = f"Start blocked; missing: {', '.join(missing)}."

        elif self.state == MissionState.TAKEOFF and inputs.autopilot.airborne:
            self._start_current_plugin()

        elif self.state == MissionState.M1_SURVEY_AREA_1:
            if self._route_complete(inputs, "m1_area1") and inputs.mapping.area_1_coverage_complete:
                if self.plan.include_mapping_area2:
                    self.transition_to(MissionState.M1_SURVEY_AREA_2, "M1 Area 1 route and coverage complete.")
                else:
                    self._finish_plugin(1, "M1 Area 1 accomplished.")

        elif self.state == MissionState.M1_SURVEY_AREA_2:
            if self._route_complete(inputs, "m1_area2") and inputs.mapping.area_2_coverage_complete:
                self._finish_plugin(1, "M1 Areas 1 and 2 accomplished.")

        elif self.state == MissionState.M4_SEARCH:
            sound_fix_valid = (
                inputs.sound.deadman_detected
                and inputs.sound.frequency_hz is not None
                and 2_600.0 <= inputs.sound.frequency_hz <= 3_000.0
                and inputs.sound.confidence >= 0.5
                and inputs.sound.target_fix is not None
                and inputs.sound.target_position_error_m is not None
                and inputs.sound.target_position_error_m <= 5.0
            )
            if sound_fix_valid:
                self.m4_target = inputs.sound.target_fix
                self.m4_target_is_deadman = True
            elif inputs.vision.deadman_led_detected and inputs.vision.deadman_target_fix is not None:
                self.m4_target = inputs.vision.deadman_target_fix
                self.m4_target_is_deadman = True
            elif inputs.vision.mannequin_target_fix is not None:
                self.m4_target = inputs.vision.mannequin_target_fix
                self.m4_target_is_deadman = False
            if self.m4_target is not None:
                target_type = "dead-man mannequin" if self.m4_target_is_deadman else "unconfirmed mannequin"
                self.transition_to(MissionState.M4_APPROACH, f"Located {target_type}.")

        elif self.state == MissionState.M4_APPROACH and self._route_complete(inputs, "m4_target_approach"):
            self.transition_to(MissionState.M4_DESCEND, "M4 approach waypoint reached.")

        elif self.state == MissionState.M4_DESCEND:
            error = inputs.vision.drop_horizontal_error_m
            if (
                inputs.autopilot.altitude_agl_m < 2.0
                and error is not None
                and error <= self.plan.first_aid_horizontal_goal_m
                and inputs.mechanism.first_aid_armed
            ):
                self.first_aid_release_altitude_m = inputs.autopilot.altitude_agl_m
                self.first_aid_horizontal_error_m = error
                self.transition_to(MissionState.M4_RELEASE, "Strict <2 m release gate satisfied.")

        elif self.state == MissionState.M4_RELEASE and inputs.mechanism.first_aid_release_complete:
            self._finish_plugin(4, "M4 first-aid release accomplished.")

        elif self.state == MissionState.RETURN_HOME and self._route_complete(inputs, "home"):
            self.transition_to(MissionState.PRECISION_LANDING, "Home route complete.")

        elif self.state == MissionState.PRECISION_LANDING and inputs.autopilot.landed:
            self.landed_at_s = inputs.now_s
            self.validated_missions = set(self.accomplished_missions)
            if 1 in self.enabled_missions:
                self.transition_to(MissionState.POST_FLIGHT_EXPORT, "Safe landing validates accomplished missions; export M1.")
            else:
                self.transition_to(MissionState.COMPLETE, "Safe landing validates Mission 4.")

        elif self.state == MissionState.POST_FLIGHT_EXPORT:
            if inputs.operator.post_flight_export_complete and inputs.mapping.map_export_ready:
                self.transition_to(MissionState.COMPLETE, "M1 map and vehicle table export confirmed.")
            elif inputs.operator.post_flight_export_complete:
                self.last_transition_reason = "Submission reported, but mapping export is not ready."

        return self._build_output(inputs)

    def _return_margin_reached(self, now_s: float) -> bool:
        return self.started_at_s is not None and (
            now_s - self.started_at_s >= self.plan.slot_duration_s - self.plan.return_time_margin_s
        )

    def _output(self, **overrides) -> FSMOutput:
        remaining = None
        if self.started_at_s is not None:
            remaining = max(0.0, self.plan.slot_duration_s - (self.latest_now_s - self.started_at_s))
        values = {
            "state": self.state,
            "profile_missions": self.enabled_missions,
            "active_mission": None,
            "flight_command": FlightCommand.HOLD,
            "accomplished_missions": tuple(sorted(self.accomplished_missions)),
            "validated_missions": tuple(sorted(self.validated_missions)),
            "accepted_vehicle_count": len(self.vehicle_records),
            "slot_deadline_remaining_s": remaining,
            "status": self.last_transition_reason,
        }
        values.update({key: value for key, value in overrides.items() if value is not None})
        return FSMOutput(**values)

    def _route_output(
        self,
        route_id: str,
        mission: Optional[int],
        vision: VisionMode = VisionMode.IDLE,
        sound: SoundMode = SoundMode.OFF,
        mechanism: MechanismCommand = MechanismCommand.SAFE,
        repeat: bool = False,
    ) -> FSMOutput:
        return self._output(
            active_mission=mission,
            flight_command=FlightCommand.FOLLOW_ROUTE,
            route_id=route_id,
            route=self.plan.route(route_id),
            route_repeat=repeat,
            vision_mode=vision,
            sound_mode=sound,
            mechanism_command=mechanism,
        )

    def _build_output(self, inputs: FSMInput) -> FSMOutput:
        state = self.state
        if state == MissionState.TAKEOFF:
            return self._output(
                flight_command=FlightCommand.TAKEOFF,
                target_altitude_agl_m=self.plan.takeoff_altitude_m,
                mechanism_command=MechanismCommand.ARM_FIRST_AID if 4 in self.enabled_missions else MechanismCommand.SAFE,
            )
        if state == MissionState.M1_SURVEY_AREA_1:
            return self._route_output("m1_area1", 1, VisionMode.MAP_AND_IDENTIFY_VEHICLES)
        if state == MissionState.M1_SURVEY_AREA_2:
            return self._route_output("m1_area2", 1, VisionMode.MAP_AND_IDENTIFY_VEHICLES)
        if state == MissionState.M4_SEARCH:
            return self._route_output(
                "m4_deadman_search",
                4,
                VisionMode.MANNEQUIN_AND_LED_SEARCH,
                SoundMode.DEADMAN_2_6_TO_3_0_KHZ,
                MechanismCommand.ARM_FIRST_AID,
                repeat=True,
            )
        if state == MissionState.M4_APPROACH:
            assert self.m4_target is not None
            approach = GeoPoint(
                self.m4_target.latitude_deg,
                self.m4_target.longitude_deg,
                self.plan.deadman_approach_altitude_m,
            )
            return self._output(
                active_mission=4,
                flight_command=FlightCommand.FOLLOW_ROUTE,
                route_id="m4_target_approach",
                route=(Waypoint("m4_target_approach", approach, 1.0),),
                vision_mode=VisionMode.FIRST_AID_DROP_ALIGNMENT,
                sound_mode=SoundMode.DEADMAN_2_6_TO_3_0_KHZ,
                mechanism_command=MechanismCommand.ARM_FIRST_AID,
            )
        if state == MissionState.M4_DESCEND:
            return self._output(
                active_mission=4,
                flight_command=FlightCommand.DESCEND_TO_ALTITUDE,
                target_altitude_agl_m=self.plan.first_aid_drop_altitude_m,
                vision_mode=VisionMode.FIRST_AID_DROP_ALIGNMENT,
                mechanism_command=MechanismCommand.ARM_FIRST_AID,
            )
        if state == MissionState.M4_RELEASE:
            return self._output(
                active_mission=4,
                flight_command=FlightCommand.HOLD_POSITION,
                vision_mode=VisionMode.FIRST_AID_DROP_ALIGNMENT,
                mechanism_command=MechanismCommand.RELEASE_FIRST_AID,
            )
        if state == MissionState.RETURN_HOME:
            return self._route_output("home", None)
        if state == MissionState.PRECISION_LANDING:
            marker_airframe = inputs.autopilot.airframe_type.lower() in {"multicopter", "multirotor", "vtol"}
            marker_ready = inputs.vision.landing_marker_visible or not marker_airframe
            return self._output(
                flight_command=FlightCommand.PRECISION_LAND if marker_ready else FlightCommand.HOLD_POSITION,
                vision_mode=VisionMode.ARUCO_ID0_PRECISION_LANDING if marker_airframe else VisionMode.IDLE,
                status="Landing controller owns the descent; mission validation waits for landed=true.",
            )
        if state == MissionState.POST_FLIGHT_EXPORT:
            remaining = None
            if self.landed_at_s is not None:
                remaining = max(0.0, 300.0 - (inputs.now_s - self.landed_at_s))
            return self._output(
                flight_command=FlightCommand.DISARMED,
                export_results=True,
                report_deadline_remaining_s=remaining,
            )
        if state == MissionState.COMPLETE:
            return self._output(flight_command=FlightCommand.DISARMED)
        if state == MissionState.EMERGENCY_LAND:
            return self._output(
                flight_command=FlightCommand.EMERGENCY_LAND,
                mechanism_command=MechanismCommand.SAFE,
            )
        return self._output(flight_command=FlightCommand.HOLD)

    def result_payload(self) -> dict:
        return {
            "profile_missions": list(self.enabled_missions),
            "accomplished_missions": sorted(self.accomplished_missions),
            "validated_missions": sorted(self.validated_missions),
            "mission_1": {
                "map_artifact_uri": self.latest_map_artifact_uri,
                "accepted_vehicle_count": len(self.vehicle_records),
                "vehicles": [asdict(record) for record in self.vehicle_records.values()],
            }
            if 1 in self.enabled_missions
            else None,
            "mission_4": {
                "target_is_deadman": self.m4_target_is_deadman,
                "release_altitude_agl_m": self.first_aid_release_altitude_m,
                "horizontal_error_m": self.first_aid_horizontal_error_m,
            }
            if 4 in self.enabled_missions
            else None,
        }

    def write_result_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.result_payload(), indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IMAV 2026 Outdoor M1/M4 pluggable FSM")
    parser.add_argument("--plan", required=True, help="Waypoint plan JSON")
    parser.add_argument(
        "--missions",
        nargs="+",
        type=int,
        choices=(1, 4),
        help="Override plan profile: --missions 1, --missions 4, or --missions 1 4",
    )
    parser.add_argument("--validate-plan", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        plan = load_plan_json(args.plan, args.missions)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise SystemExit(f"[PLAN ERROR] {error}") from error
    print(f"Plan valid for profile {plan.enabled_missions}: {len(plan.routes)} routes loaded")
    if not args.validate_plan:
        fsm = OutdoorMissionFSM(plan)
        print(f"FSM ready in {fsm.state.value}; call step() from the middleware loop.")
