"""Macro state machine and typed module ports for IMAV 2026 Outdoor.

The FSM intentionally does not contain object detection, sound localisation,
mechanism control loops, or low-level flight control.  It requests those
capabilities through typed output ports and consumes their results through typed
input ports.
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
    M3_TRANSIT_WATER_SOURCE = "M3_TRANSIT_WATER_SOURCE"
    M3_COLLECT_WATER = "M3_COLLECT_WATER"
    M3_TRANSIT_DROP_TARGET = "M3_TRANSIT_DROP_TARGET"
    M3_ALIGN_WATER_TARGET = "M3_ALIGN_WATER_TARGET"
    M3_RELEASE_WATER = "M3_RELEASE_WATER"
    M1_SURVEY_AREA_1 = "M1_SURVEY_AREA_1"
    M1_SURVEY_AREA_2 = "M1_SURVEY_AREA_2"
    M2_SEARCH_HOTSPOTS = "M2_SEARCH_HOTSPOTS"
    M4_SEARCH_DEADMAN = "M4_SEARCH_DEADMAN"
    M4_APPROACH_DEADMAN = "M4_APPROACH_DEADMAN"
    M4_DESCEND_FOR_DROP = "M4_DESCEND_FOR_DROP"
    M4_RELEASE_FIRST_AID = "M4_RELEASE_FIRST_AID"
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
    WATER_SOURCE_ALIGNMENT = "WATER_SOURCE_ALIGNMENT"
    WATER_TARGET_ALIGNMENT = "WATER_TARGET_ALIGNMENT"
    MAP_AND_IDENTIFY_VEHICLES = "MAP_AND_IDENTIFY_VEHICLES"
    THERMAL_HOTSPOT_LOCALISATION = "THERMAL_HOTSPOT_LOCALISATION"
    MANNEQUIN_AND_LED_SEARCH = "MANNEQUIN_AND_LED_SEARCH"
    FIRST_AID_DROP_ALIGNMENT = "FIRST_AID_DROP_ALIGNMENT"
    ARUCO_ID0_PRECISION_LANDING = "ARUCO_ID0_PRECISION_LANDING"


class SoundMode(str, Enum):
    OFF = "OFF"
    DEADMAN_2_6_TO_3_0_KHZ = "DEADMAN_2_6_TO_3_0_KHZ"


class MechanismCommand(str, Enum):
    SAFE = "SAFE"
    PREPARE_WATER_COLLECTION = "PREPARE_WATER_COLLECTION"
    COLLECT_WATER = "COLLECT_WATER"
    RELEASE_WATER = "RELEASE_WATER"
    ARM_FIRST_AID = "ARM_FIRST_AID"
    RELEASE_FIRST_AID = "RELEASE_FIRST_AID"


@dataclass(frozen=True)
class LocatedObservation:
    """Detection fused with GPS localisation.

    position_error_m is the localisation module's conservative horizontal error
    estimate.  Mission 1 and 2 observations over 5 m are retained by perception
    but are not accepted by the FSM as scoring-valid records.
    """

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
    flight_time_s: float = 0.0
    battery_capacity_mah: Optional[float] = None
    battery_max_voltage_v: Optional[float] = None
    airframe_type: str = "multicopter"


@dataclass(frozen=True)
class VisionInput:
    detection_ready: bool = False
    localisation_and_calibration_ready: bool = False
    vehicle_fixes: Tuple[LocatedObservation, ...] = ()
    hotspot_fixes: Tuple[LocatedObservation, ...] = ()
    water_source_aligned: bool = False
    water_target_aligned: bool = False
    deadman_led_detected: bool = False
    deadman_target_fix: Optional[GeoPoint] = None
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
    water_collection_complete: bool = False
    water_volume_l: float = 0.0
    water_release_complete: bool = False
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
    completed_missions: Tuple[int, ...] = ()
    accepted_vehicle_count: int = 0
    accepted_hotspot_count: int = 0
    report_deadline_remaining_s: Optional[float] = None
    slot_deadline_remaining_s: Optional[float] = None
    status: str = ""


class OutdoorMissionFSM:
    """One-flight orchestrator for all four outdoor missions.

    Default spatial order: M3 -> M1 -> M2 -> M4 -> landing/export.
    The order minimises long backtracking from the takeoff zone shown in Fig. 16
    and prioritises mission completion over online route optimisation.
    """

    def __init__(self, plan: MissionPlan) -> None:
        plan.validate()
        self.plan = plan
        self.state = MissionState.PRE_FLIGHT
        self.completed_missions: set[int] = set()
        self.vehicle_records: Dict[str, LocatedObservation] = {}
        self.hotspot_records: Dict[str, LocatedObservation] = {}
        self.deadman_target: Optional[GeoPoint] = None
        self.landed_at_s: Optional[float] = None
        self.started_at_s: Optional[float] = None
        self.latest_now_s: float = 0.0
        self.collected_water_volume_l: Optional[float] = None
        self.water_released_volume_l: Optional[float] = None
        self.first_aid_release_altitude_m: Optional[float] = None
        self.first_aid_horizontal_error_m: Optional[float] = None
        self.latest_autopilot = AutopilotInput()
        self.latest_map_artifact_uri: Optional[str] = None
        self.last_transition_reason = "Waiting for start and all pre-flight readiness inputs."

    def transition_to(self, state: MissionState, reason: str) -> None:
        if state == self.state:
            return
        print(f"[FSM] {self.state.value} -> {state.value}: {reason}")
        self.state = state
        self.last_transition_reason = reason

    def _route_complete(self, inputs: FSMInput, route_id: str) -> bool:
        return inputs.autopilot.completed_route_id == route_id

    def _capture_scoring_records(self, inputs: FSMInput) -> None:
        if self.state in (MissionState.M1_SURVEY_AREA_1, MissionState.M1_SURVEY_AREA_2):
            for observation in inputs.vision.vehicle_fixes:
                if (
                    observation.confidence >= 0.5
                    and observation.position_error_m <= 5.0
                    and observation.label.strip()
                ):
                    self.vehicle_records[observation.observation_id] = observation

        if self.state == MissionState.M2_SEARCH_HOTSPOTS:
            for observation in inputs.vision.hotspot_fixes:
                if observation.confidence >= 0.5 and observation.position_error_m <= 5.0:
                    self.hotspot_records[observation.observation_id] = observation

    def _preflight_missing(self, inputs: FSMInput) -> Tuple[str, ...]:
        missing = []
        if not inputs.autopilot.preflight_ok or not inputs.autopilot.gps_ok:
            missing.append("autopilot/GPS")
        if inputs.autopilot.battery_percent <= self.plan.low_battery_return_percent:
            missing.append("battery above return reserve")
        if not inputs.vision.detection_ready:
            missing.append("vision detection")
        if not inputs.vision.localisation_and_calibration_ready:
            missing.append("vision localisation/calibration")
        if not inputs.mapping.ready:
            missing.append("mapping")
        if not inputs.sound.ready:
            missing.append("sound localisation")
        if not inputs.mechanism.ready or not inputs.mechanism.first_aid_loaded or inputs.mechanism.fault:
            missing.append("mechanism/first-aid payload")
        return tuple(missing)

    def step(self, inputs: FSMInput) -> FSMOutput:
        """Consume one complete input snapshot and return one command snapshot."""

        self._capture_scoring_records(inputs)
        self.latest_now_s = inputs.now_s
        self.latest_autopilot = inputs.autopilot
        if inputs.mapping.map_artifact_uri:
            self.latest_map_artifact_uri = inputs.mapping.map_artifact_uri

        terminal_states = {MissionState.COMPLETE, MissionState.EMERGENCY_LAND}
        if self.state not in terminal_states:
            if inputs.operator.abort or not inputs.autopilot.inside_geofence:
                self.transition_to(MissionState.EMERGENCY_LAND, "Abort or geofence violation.")
            elif (
                inputs.operator.force_return
                or inputs.autopilot.battery_percent <= self.plan.low_battery_return_percent
                or inputs.mechanism.fault
                or (
                    self.started_at_s is not None
                    and inputs.now_s - self.started_at_s
                    >= self.plan.slot_duration_s - self.plan.return_time_margin_s
                )
            ) and self.state not in (
                MissionState.PRE_FLIGHT,
                MissionState.RETURN_HOME,
                MissionState.PRECISION_LANDING,
                MissionState.POST_FLIGHT_EXPORT,
            ):
                self.transition_to(MissionState.RETURN_HOME, "Forced safe return, low battery, or mechanism fault.")

        if self.state == MissionState.PRE_FLIGHT:
            missing = self._preflight_missing(inputs)
            if inputs.operator.start and not missing:
                self.started_at_s = inputs.now_s
                self.transition_to(MissionState.TAKEOFF, "Start accepted; all modules ready.")
            elif inputs.operator.start:
                self.last_transition_reason = f"Start blocked; missing: {', '.join(missing)}."

        elif self.state == MissionState.TAKEOFF:
            if inputs.autopilot.airborne:
                self.transition_to(MissionState.M3_TRANSIT_WATER_SOURCE, "Takeoff confirmed.")

        elif self.state == MissionState.M3_TRANSIT_WATER_SOURCE:
            if self._route_complete(inputs, "m3_water_source"):
                self.transition_to(MissionState.M3_COLLECT_WATER, "Water source waypoint reached.")

        elif self.state == MissionState.M3_COLLECT_WATER:
            if inputs.mechanism.water_collection_complete and inputs.mechanism.water_volume_l > 0.0:
                self.collected_water_volume_l = min(inputs.mechanism.water_volume_l, 2.0)
                self.transition_to(
                    MissionState.M3_TRANSIT_DROP_TARGET,
                    f"Water collection confirmed ({inputs.mechanism.water_volume_l:.2f} L).",
                )

        elif self.state == MissionState.M3_TRANSIT_DROP_TARGET:
            if self._route_complete(inputs, "m3_water_target"):
                self.transition_to(MissionState.M3_ALIGN_WATER_TARGET, "Water target waypoint reached.")

        elif self.state == MissionState.M3_ALIGN_WATER_TARGET:
            if inputs.vision.water_target_aligned:
                self.transition_to(MissionState.M3_RELEASE_WATER, "Vision confirmed target alignment.")

        elif self.state == MissionState.M3_RELEASE_WATER:
            if inputs.mechanism.water_release_complete:
                self.water_released_volume_l = self.collected_water_volume_l
                self.completed_missions.add(3)
                self.transition_to(MissionState.M1_SURVEY_AREA_1, "Water release confirmed; Mission 3 complete.")

        elif self.state == MissionState.M1_SURVEY_AREA_1:
            if self._route_complete(inputs, "m1_area1") and inputs.mapping.area_1_coverage_complete:
                if self.plan.include_mapping_area2:
                    self.transition_to(MissionState.M1_SURVEY_AREA_2, "Mapping Area 1 route and coverage complete.")
                else:
                    self.completed_missions.add(1)
                    self.transition_to(MissionState.M2_SEARCH_HOTSPOTS, "Mission 1 minimum area complete.")

        elif self.state == MissionState.M1_SURVEY_AREA_2:
            if self._route_complete(inputs, "m1_area2") and inputs.mapping.area_2_coverage_complete:
                self.completed_missions.add(1)
                self.transition_to(MissionState.M2_SEARCH_HOTSPOTS, "Both mapping areas complete; Mission 1 complete.")

        elif self.state == MissionState.M2_SEARCH_HOTSPOTS:
            if len(self.hotspot_records) >= 2:
                self.completed_missions.add(2)
                self.transition_to(MissionState.M4_SEARCH_DEADMAN, "Two scoring-valid hotspot fixes recorded.")

        elif self.state == MissionState.M4_SEARCH_DEADMAN:
            sound_fix_valid = (
                inputs.sound.deadman_detected
                and inputs.sound.target_fix is not None
                and inputs.sound.confidence >= 0.5
                and inputs.sound.target_position_error_m is not None
                and inputs.sound.target_position_error_m <= 5.0
                and inputs.sound.frequency_hz is not None
                and 2_600.0 <= inputs.sound.frequency_hz <= 3_000.0
            )
            if sound_fix_valid:
                self.deadman_target = inputs.sound.target_fix
            elif inputs.vision.deadman_led_detected and inputs.vision.deadman_target_fix is not None:
                self.deadman_target = inputs.vision.deadman_target_fix
            if self.deadman_target is not None:
                self.transition_to(MissionState.M4_APPROACH_DEADMAN, "Dead-man target localised.")

        elif self.state == MissionState.M4_APPROACH_DEADMAN:
            if self._route_complete(inputs, "m4_deadman_approach"):
                self.transition_to(MissionState.M4_DESCEND_FOR_DROP, "Approach waypoint reached.")

        elif self.state == MissionState.M4_DESCEND_FOR_DROP:
            horizontal_error = inputs.vision.drop_horizontal_error_m
            if (
                inputs.autopilot.altitude_agl_m <= 2.0
                and horizontal_error is not None
                and horizontal_error <= self.plan.first_aid_horizontal_goal_m
                and inputs.mechanism.first_aid_armed
            ):
                self.first_aid_release_altitude_m = inputs.autopilot.altitude_agl_m
                self.first_aid_horizontal_error_m = horizontal_error
                self.transition_to(
                    MissionState.M4_RELEASE_FIRST_AID,
                    "Release gate satisfied: altitude <=2 m, target aligned, mechanism armed.",
                )

        elif self.state == MissionState.M4_RELEASE_FIRST_AID:
            if inputs.mechanism.first_aid_release_complete:
                self.completed_missions.add(4)
                self.transition_to(MissionState.RETURN_HOME, "First-aid release confirmed; Mission 4 complete.")

        elif self.state == MissionState.RETURN_HOME:
            if self._route_complete(inputs, "home"):
                self.transition_to(MissionState.PRECISION_LANDING, "Home route complete; request ID 0 landing.")

        elif self.state == MissionState.PRECISION_LANDING:
            if inputs.autopilot.landed:
                self.landed_at_s = inputs.now_s
                self.transition_to(MissionState.POST_FLIGHT_EXPORT, "Landing confirmed; start five-minute export timer.")

        elif self.state == MissionState.POST_FLIGHT_EXPORT:
            if inputs.operator.post_flight_export_complete and inputs.mapping.map_export_ready:
                self.transition_to(MissionState.COMPLETE, "Mission 1/2 result files submitted/exported.")
            elif inputs.operator.post_flight_export_complete:
                self.last_transition_reason = "Submission confirmation received, but the final map is not export-ready."

        return self._build_output(inputs)

    def _route_output(
        self,
        route_id: str,
        active_mission: Optional[int],
        vision_mode: VisionMode = VisionMode.IDLE,
        sound_mode: SoundMode = SoundMode.OFF,
        mechanism_command: MechanismCommand = MechanismCommand.SAFE,
        route_repeat: bool = False,
        status: Optional[str] = None,
    ) -> FSMOutput:
        return self._output(
            active_mission=active_mission,
            flight_command=FlightCommand.FOLLOW_ROUTE,
            route_id=route_id,
            route=self.plan.route(route_id),
            vision_mode=vision_mode,
            sound_mode=sound_mode,
            mechanism_command=mechanism_command,
            route_repeat=route_repeat,
            status=status,
        )

    def _output(self, **overrides) -> FSMOutput:
        values = {
            "state": self.state,
            "active_mission": None,
            "flight_command": FlightCommand.HOLD,
            "completed_missions": tuple(sorted(self.completed_missions)),
            "accepted_vehicle_count": len(self.vehicle_records),
            "accepted_hotspot_count": len(self.hotspot_records),
            "slot_deadline_remaining_s": self._slot_remaining_s(),
            "status": self.last_transition_reason,
        }
        values.update({key: value for key, value in overrides.items() if value is not None})
        return FSMOutput(**values)

    def _build_output(self, inputs: FSMInput) -> FSMOutput:
        state = self.state
        if state == MissionState.PRE_FLIGHT:
            return self._output(flight_command=FlightCommand.HOLD, mechanism_command=MechanismCommand.SAFE)
        if state == MissionState.TAKEOFF:
            return self._output(
                flight_command=FlightCommand.TAKEOFF,
                target_altitude_agl_m=self.plan.takeoff_altitude_m,
                mechanism_command=MechanismCommand.ARM_FIRST_AID,
            )
        if state == MissionState.M3_TRANSIT_WATER_SOURCE:
            return self._route_output(
                "m3_water_source",
                3,
                VisionMode.WATER_SOURCE_ALIGNMENT,
                mechanism_command=MechanismCommand.PREPARE_WATER_COLLECTION,
            )
        if state == MissionState.M3_COLLECT_WATER:
            return self._output(
                active_mission=3,
                flight_command=FlightCommand.HOLD_POSITION,
                vision_mode=VisionMode.WATER_SOURCE_ALIGNMENT,
                mechanism_command=(
                    MechanismCommand.COLLECT_WATER
                    if inputs.vision.water_source_aligned
                    else MechanismCommand.PREPARE_WATER_COLLECTION
                ),
                status=(
                    f"Collecting water; goal {self.plan.water_goal_l:.2f} L."
                    if inputs.vision.water_source_aligned
                    else "Waiting for vision alignment over the water source."
                ),
            )
        if state == MissionState.M3_TRANSIT_DROP_TARGET:
            return self._route_output("m3_water_target", 3, VisionMode.WATER_TARGET_ALIGNMENT)
        if state == MissionState.M3_ALIGN_WATER_TARGET:
            return self._output(
                active_mission=3,
                flight_command=FlightCommand.HOLD_POSITION,
                vision_mode=VisionMode.WATER_TARGET_ALIGNMENT,
                status="Waiting for alignment over the 1.5 x 1.5 m red target.",
            )
        if state == MissionState.M3_RELEASE_WATER:
            return self._output(
                active_mission=3,
                flight_command=FlightCommand.HOLD_POSITION,
                vision_mode=VisionMode.WATER_TARGET_ALIGNMENT,
                mechanism_command=MechanismCommand.RELEASE_WATER,
            )
        if state == MissionState.M1_SURVEY_AREA_1:
            return self._route_output("m1_area1", 1, VisionMode.MAP_AND_IDENTIFY_VEHICLES)
        if state == MissionState.M1_SURVEY_AREA_2:
            return self._route_output("m1_area2", 1, VisionMode.MAP_AND_IDENTIFY_VEHICLES)
        if state == MissionState.M2_SEARCH_HOTSPOTS:
            return self._route_output(
                "m2_fire_search",
                2,
                VisionMode.THERMAL_HOTSPOT_LOCALISATION,
                route_repeat=True,
                status="Repeat the deterministic search route until two <=5 m GPS fixes are accepted.",
            )
        if state == MissionState.M4_SEARCH_DEADMAN:
            return self._route_output(
                "m4_deadman_search",
                4,
                VisionMode.MANNEQUIN_AND_LED_SEARCH,
                SoundMode.DEADMAN_2_6_TO_3_0_KHZ,
                MechanismCommand.ARM_FIRST_AID,
                route_repeat=True,
            )
        if state == MissionState.M4_APPROACH_DEADMAN:
            assert self.deadman_target is not None
            approach_position = GeoPoint(
                latitude_deg=self.deadman_target.latitude_deg,
                longitude_deg=self.deadman_target.longitude_deg,
                altitude_agl_m=self.plan.deadman_approach_altitude_m,
            )
            dynamic_route = (
                Waypoint("m4_deadman_approach", approach_position, acceptance_radius_m=1.0),
            )
            return self._output(
                active_mission=4,
                flight_command=FlightCommand.FOLLOW_ROUTE,
                route_id="m4_deadman_approach",
                route=dynamic_route,
                vision_mode=VisionMode.FIRST_AID_DROP_ALIGNMENT,
                sound_mode=SoundMode.DEADMAN_2_6_TO_3_0_KHZ,
                mechanism_command=MechanismCommand.ARM_FIRST_AID,
            )
        if state == MissionState.M4_DESCEND_FOR_DROP:
            return self._output(
                active_mission=4,
                flight_command=FlightCommand.DESCEND_TO_ALTITUDE,
                target_altitude_agl_m=self.plan.first_aid_drop_altitude_m,
                vision_mode=VisionMode.FIRST_AID_DROP_ALIGNMENT,
                sound_mode=SoundMode.DEADMAN_2_6_TO_3_0_KHZ,
                mechanism_command=MechanismCommand.ARM_FIRST_AID,
            )
        if state == MissionState.M4_RELEASE_FIRST_AID:
            return self._output(
                active_mission=4,
                flight_command=FlightCommand.HOLD_POSITION,
                vision_mode=VisionMode.FIRST_AID_DROP_ALIGNMENT,
                mechanism_command=MechanismCommand.RELEASE_FIRST_AID,
            )
        if state == MissionState.RETURN_HOME:
            return self._route_output("home", None, mechanism_command=MechanismCommand.SAFE)
        if state == MissionState.PRECISION_LANDING:
            marker_landing = inputs.autopilot.airframe_type.strip().lower() in {
                "multicopter",
                "multirotor",
                "vtol",
            }
            return self._output(
                flight_command=(
                    FlightCommand.PRECISION_LAND
                    if inputs.vision.landing_marker_visible or not marker_landing
                    else FlightCommand.HOLD_POSITION
                ),
                vision_mode=(VisionMode.ARUCO_ID0_PRECISION_LANDING if marker_landing else VisionMode.IDLE),
                mechanism_command=MechanismCommand.SAFE,
                status=(
                    "ArUco ID 0 acquired; landing controller owns the descent loop."
                    if marker_landing and inputs.vision.landing_marker_visible
                    else "Fixed-wing landing-area approach; marker landing is not applicable."
                    if not marker_landing
                    else "Hold above landing zone and search for 5x5 ArUco ID 0."
                ),
            )
        if state == MissionState.POST_FLIGHT_EXPORT:
            remaining = None
            if self.landed_at_s is not None:
                remaining = max(0.0, 300.0 - (inputs.now_s - self.landed_at_s))
            return self._output(
                flight_command=FlightCommand.DISARMED,
                export_results=True,
                report_deadline_remaining_s=remaining,
                status="Export map, vehicle table, and two hotspot GPS coordinates within five minutes.",
            )
        if state == MissionState.COMPLETE:
            return self._output(flight_command=FlightCommand.DISARMED, status="All orchestration finished.")
        return self._output(
            flight_command=FlightCommand.EMERGENCY_LAND,
            mechanism_command=MechanismCommand.SAFE,
            status="Emergency landing requested; low-level safety controller retains final authority.",
        )

    def result_payload(self) -> dict:
        """Machine-readable payload for the post-flight reporting module."""

        return {
            "completed_missions": sorted(self.completed_missions),
            "mission_1": {
                "map_artifact_uri": self.latest_map_artifact_uri,
                "accepted_vehicle_count": len(self.vehicle_records),
            },
            "mission_2": {
                "accepted_hotspot_count": len(self.hotspot_records),
                "flight_time_s": self.latest_autopilot.flight_time_s,
                "battery_capacity_mah": self.latest_autopilot.battery_capacity_mah,
                "battery_max_voltage_v": self.latest_autopilot.battery_max_voltage_v,
                "airframe_type": self.latest_autopilot.airframe_type,
            },
            "mission_3": {"released_water_volume_l": self.water_released_volume_l},
            "mission_4": {
                "release_altitude_agl_m": self.first_aid_release_altitude_m,
                "horizontal_error_m": self.first_aid_horizontal_error_m,
            },
            "vehicles": [asdict(record) for record in self.vehicle_records.values()],
            "hotspots": [asdict(record) for record in self.hotspot_records.values()],
        }

    def _slot_remaining_s(self) -> Optional[float]:
        if self.started_at_s is None:
            return None
        return max(0.0, self.plan.slot_duration_s - (self.latest_now_s - self.started_at_s))

    def write_result_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.result_payload(), indent=2) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IMAV 2026 outdoor macro FSM")
    parser.add_argument("--plan", required=True, help="Day-of-competition waypoint plan JSON")
    parser.add_argument("--validate-plan", action="store_true", help="Validate the route table and exit")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        mission_plan = load_plan_json(args.plan)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise SystemExit(f"[PLAN ERROR] {error}") from error
    print(f"Plan valid: {len(mission_plan.routes)} routes")
    if not args.validate_plan:
        fsm = OutdoorMissionFSM(mission_plan)
        print(f"FSM ready in state {fsm.state.value}; integrate step() with the middleware loop.")
