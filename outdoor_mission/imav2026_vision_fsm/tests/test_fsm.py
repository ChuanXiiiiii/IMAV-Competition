import sys
import unittest
from dataclasses import replace
from pathlib import Path


MODULE_DIR = Path(__file__).parents[1]
sys.path.insert(0, str(MODULE_DIR))

from imav2026_outdoor_fsm import (  # noqa: E402
    AutopilotInput,
    FSMInput,
    FlightCommand,
    LocatedObservation,
    MappingInput,
    MechanismInput,
    MissionState,
    OperatorInput,
    OutdoorMissionFSM,
    SoundInput,
    VisionInput,
    VisionMode,
)
from simple_waypoint_planner import GeoPoint, MissionPlan, Waypoint  # noqa: E402


def point(index: int, altitude: float = 30.0) -> GeoPoint:
    return GeoPoint(48.806 + index * 0.00001, 7.852 + index * 0.00001, altitude)


def wp(name: str, index: int) -> Waypoint:
    return Waypoint(name, point(index))


def make_plan(missions=(1, 4), include_area2=True, routes=None) -> MissionPlan:
    all_routes = routes or {
        "m1_area1": (wp("m1a", 1), wp("m1b", 2)),
        "m1_area2": (wp("m1c", 3), wp("m1d", 4)),
        "m4_deadman_search": (wp("m4a", 5), wp("m4b", 6)),
        "home": (wp("home", 0),),
    }
    return MissionPlan(
        routes=all_routes,
        enabled_missions=tuple(missions),
        include_mapping_area2=include_area2,
    )


READY_AP = AutopilotInput(preflight_ok=True, gps_ok=True, inside_geofence=True, landed=True)
READY_VISION = VisionInput(detection_ready=True, localisation_and_calibration_ready=True)
READY_MAPPING = MappingInput(ready=True)
READY_SOUND = SoundInput(ready=True)
READY_MECH = MechanismInput(
    ready=True,
    first_aid_loaded=True,
    first_aid_kit_verified=True,
)


def snap(**overrides) -> FSMInput:
    values = {
        "now_s": 0.0,
        "autopilot": READY_AP,
        "vision": READY_VISION,
        "mapping": READY_MAPPING,
        "sound": READY_SOUND,
        "mechanism": READY_MECH,
    }
    values.update(overrides)
    return FSMInput(**values)


def start_and_takeoff(fsm: OutdoorMissionFSM, **overrides):
    fsm.step(snap(operator=OperatorInput(start=True), **overrides))
    return fsm.step(snap(autopilot=replace(READY_AP, airborne=True, landed=False), **overrides))


def finish_landing(fsm: OutdoorMissionFSM, now=100.0):
    fsm.step(snap(autopilot=replace(READY_AP, completed_route_id="home", landed=False)))
    return fsm.step(
        snap(
            now_s=now,
            autopilot=replace(READY_AP, landed=True),
            vision=replace(READY_VISION, landing_marker_visible=True),
        )
    )


class PluggableProfilesTests(unittest.TestCase):
    def test_mission_1_only_does_not_require_sound_or_mechanism(self):
        fsm = OutdoorMissionFSM(make_plan((1,)))
        output = start_and_takeoff(
            fsm,
            sound=SoundInput(ready=False),
            mechanism=MechanismInput(ready=False),
        )
        self.assertEqual(output.state, MissionState.M1_SURVEY_AREA_1)
        self.assertEqual(output.profile_missions, (1,))

        vehicle = LocatedObservation("v1", "67-CCF-M-ING", point(10), 0.9, 3.0)
        fsm.step(snap(vision=replace(READY_VISION, vehicle_fixes=(vehicle,))))
        fsm.step(
            snap(
                autopilot=replace(READY_AP, completed_route_id="m1_area1"),
                mapping=replace(READY_MAPPING, area_1_coverage_complete=True),
            )
        )
        output = fsm.step(
            snap(
                autopilot=replace(READY_AP, completed_route_id="m1_area2"),
                mapping=replace(READY_MAPPING, area_2_coverage_complete=True),
            )
        )
        self.assertEqual(output.state, MissionState.RETURN_HOME)
        self.assertEqual(output.accomplished_missions, (1,))
        self.assertEqual(output.validated_missions, ())

        output = finish_landing(fsm)
        self.assertEqual(output.state, MissionState.POST_FLIGHT_EXPORT)
        self.assertEqual(output.validated_missions, (1,))
        output = fsm.step(
            snap(
                operator=OperatorInput(post_flight_export_complete=True),
                mapping=replace(READY_MAPPING, map_export_ready=True, map_artifact_uri="map.tif"),
            )
        )
        self.assertEqual(output.state, MissionState.COMPLETE)
        self.assertEqual(fsm.result_payload()["mission_1"]["accepted_vehicle_count"], 1)
        self.assertIsNone(fsm.result_payload()["mission_4"])

    def test_mission_4_only_does_not_require_mapping(self):
        fsm = OutdoorMissionFSM(make_plan((4,)))
        output = start_and_takeoff(fsm, mapping=MappingInput(ready=False))
        self.assertEqual(output.state, MissionState.M4_SEARCH)
        self.assertEqual(output.profile_missions, (4,))

        target = point(20, 0.0)
        output = fsm.step(
            snap(
                sound=SoundInput(
                    ready=True,
                    deadman_detected=True,
                    frequency_hz=2800.0,
                    confidence=0.9,
                    target_fix=target,
                    target_position_error_m=2.0,
                )
            )
        )
        self.assertEqual(output.state, MissionState.M4_APPROACH)
        self.assertEqual(output.route_id, "m4_target_approach")

        fsm.step(snap(autopilot=replace(READY_AP, completed_route_id="m4_target_approach")))
        output = fsm.step(
            snap(
                autopilot=replace(READY_AP, altitude_agl_m=1.5),
                vision=replace(READY_VISION, drop_horizontal_error_m=0.4),
                mechanism=replace(READY_MECH, first_aid_armed=True),
            )
        )
        self.assertEqual(output.state, MissionState.M4_RELEASE)
        output = fsm.step(snap(mechanism=replace(READY_MECH, first_aid_release_complete=True)))
        self.assertEqual(output.accomplished_missions, (4,))
        self.assertEqual(output.validated_missions, ())

        output = finish_landing(fsm)
        self.assertEqual(output.state, MissionState.COMPLETE)
        self.assertEqual(output.validated_missions, (4,))
        self.assertIsNone(fsm.result_payload()["mission_1"])

    def test_combined_profile_runs_m1_then_m4_then_one_landing(self):
        fsm = OutdoorMissionFSM(make_plan((1, 4), include_area2=False))
        output = start_and_takeoff(fsm)
        self.assertEqual(output.state, MissionState.M1_SURVEY_AREA_1)

        output = fsm.step(
            snap(
                autopilot=replace(READY_AP, completed_route_id="m1_area1"),
                mapping=replace(READY_MAPPING, area_1_coverage_complete=True),
            )
        )
        self.assertEqual(output.state, MissionState.M4_SEARCH)
        self.assertEqual(output.accomplished_missions, (1,))

        target = point(30, 0.0)
        fsm.step(
            snap(
                vision=replace(
                    READY_VISION,
                    deadman_led_detected=True,
                    deadman_target_fix=target,
                )
            )
        )
        fsm.step(snap(autopilot=replace(READY_AP, completed_route_id="m4_target_approach")))
        fsm.step(
            snap(
                autopilot=replace(READY_AP, altitude_agl_m=1.0),
                vision=replace(READY_VISION, drop_horizontal_error_m=0.5),
                mechanism=replace(READY_MECH, first_aid_armed=True),
            )
        )
        output = fsm.step(snap(mechanism=replace(READY_MECH, first_aid_release_complete=True)))
        self.assertEqual(output.state, MissionState.RETURN_HOME)
        self.assertEqual(output.accomplished_missions, (1, 4))
        self.assertEqual(output.validated_missions, ())

        output = finish_landing(fsm)
        self.assertEqual(output.validated_missions, (1, 4))
        self.assertEqual(output.state, MissionState.POST_FLIGHT_EXPORT)


class RuleAndSafetyTests(unittest.TestCase):
    def test_m4_preflight_requires_verified_v4_payload(self):
        fsm = OutdoorMissionFSM(make_plan((4,)))
        output = fsm.step(
            snap(
                operator=OperatorInput(start=True),
                mechanism=replace(READY_MECH, first_aid_kit_verified=False),
            )
        )
        self.assertEqual(output.state, MissionState.PRE_FLIGHT)
        self.assertIn("V4-1", output.status)

    def test_exactly_two_metres_does_not_pass_strict_v4_drop_gate(self):
        fsm = OutdoorMissionFSM(make_plan((4,)))
        fsm.state = MissionState.M4_DESCEND
        output = fsm.step(
            snap(
                autopilot=replace(READY_AP, altitude_agl_m=2.0),
                vision=replace(READY_VISION, drop_horizontal_error_m=0.4),
                mechanism=replace(READY_MECH, first_aid_armed=True),
            )
        )
        self.assertEqual(output.state, MissionState.M4_DESCEND)

    def test_m4_can_target_unconfirmed_mannequin_for_base_score(self):
        fsm = OutdoorMissionFSM(make_plan((4,)))
        fsm.state = MissionState.M4_SEARCH
        output = fsm.step(snap(vision=replace(READY_VISION, mannequin_target_fix=point(9))))
        self.assertEqual(output.state, MissionState.M4_APPROACH)
        self.assertFalse(fsm.m4_target_is_deadman)

    def test_invalid_vehicle_fix_is_not_counted(self):
        fsm = OutdoorMissionFSM(make_plan((1,)))
        fsm.state = MissionState.M1_SURVEY_AREA_1
        invalid = LocatedObservation("v1", "VBL", point(1), 0.9, 5.1)
        output = fsm.step(snap(vision=replace(READY_VISION, vehicle_fixes=(invalid,))))
        self.assertEqual(output.accepted_vehicle_count, 0)

    def test_geofence_violation_requests_emergency_land(self):
        fsm = OutdoorMissionFSM(make_plan((1,)))
        fsm.state = MissionState.M1_SURVEY_AREA_1
        output = fsm.step(snap(autopilot=replace(READY_AP, inside_geofence=False)))
        self.assertEqual(output.state, MissionState.EMERGENCY_LAND)
        self.assertEqual(output.flight_command, FlightCommand.EMERGENCY_LAND)

    def test_time_margin_forces_return(self):
        fsm = OutdoorMissionFSM(make_plan((1, 4)))
        fsm.state = MissionState.M4_SEARCH
        fsm.started_at_s = 0.0
        output = fsm.step(snap(now_s=1620.0))
        self.assertEqual(output.state, MissionState.RETURN_HOME)

    def test_plan_validation_is_profile_specific(self):
        m1_routes = {"m1_area1": (wp("m1", 1),), "home": (wp("home", 0),)}
        make_plan((1,), include_area2=False, routes=m1_routes).validate()
        with self.assertRaises(ValueError):
            make_plan((4,), routes=m1_routes).validate()

    def test_multicopter_holds_until_aruco_is_visible(self):
        fsm = OutdoorMissionFSM(make_plan((1,)))
        fsm.state = MissionState.PRECISION_LANDING
        airborne = replace(READY_AP, airborne=True, landed=False)
        output = fsm.step(snap(autopilot=airborne))
        self.assertEqual(output.flight_command, FlightCommand.HOLD_POSITION)
        self.assertEqual(output.vision_mode, VisionMode.ARUCO_ID0_PRECISION_LANDING)


if __name__ == "__main__":
    unittest.main()
