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
from simple_waypoint_planner import GeoPoint, MissionPlan, Waypoint, generate_lawnmower_route  # noqa: E402


def point(index: int, altitude: float = 30.0) -> GeoPoint:
    return GeoPoint(48.806 + index * 0.00001, 7.852 + index * 0.00001, altitude)


def waypoint(name: str, index: int) -> Waypoint:
    return Waypoint(name, point(index))


def plan(include_area2: bool = True) -> MissionPlan:
    routes = {
        "m3_water_source": (waypoint("water_source", 1),),
        "m3_water_target": (waypoint("water_target", 2),),
        "m1_area1": (waypoint("map1_a", 3), waypoint("map1_b", 4)),
        "m2_fire_search": (waypoint("fire_a", 5), waypoint("fire_b", 6)),
        "m4_deadman_search": (waypoint("deadman_a", 7), waypoint("deadman_b", 8)),
        "home": (waypoint("home", 0),),
    }
    if include_area2:
        routes["m1_area2"] = (waypoint("map2_a", 9), waypoint("map2_b", 10))
    return MissionPlan(routes=routes, include_mapping_area2=include_area2)


READY_AUTOPILOT = AutopilotInput(preflight_ok=True, gps_ok=True, inside_geofence=True)
READY_VISION = VisionInput(detection_ready=True, localisation_and_calibration_ready=True)
READY_MAPPING = MappingInput(ready=True)
READY_SOUND = SoundInput(ready=True)
READY_MECHANISM = MechanismInput(ready=True, first_aid_loaded=True)


def snapshot(**changes) -> FSMInput:
    values = {
        "now_s": 0.0,
        "autopilot": READY_AUTOPILOT,
        "vision": READY_VISION,
        "mapping": READY_MAPPING,
        "sound": READY_SOUND,
        "mechanism": READY_MECHANISM,
    }
    values.update(changes)
    return FSMInput(**values)


class FullMissionFlowTests(unittest.TestCase):
    def test_all_four_missions_in_one_flight(self):
        fsm = OutdoorMissionFSM(plan())

        output = fsm.step(snapshot(operator=OperatorInput(start=True)))
        self.assertEqual(output.state, MissionState.TAKEOFF)
        self.assertEqual(output.flight_command, FlightCommand.TAKEOFF)

        output = fsm.step(snapshot(autopilot=replace(READY_AUTOPILOT, airborne=True, landed=False)))
        self.assertEqual(output.state, MissionState.M3_TRANSIT_WATER_SOURCE)
        self.assertEqual(output.route_id, "m3_water_source")

        fsm.step(snapshot(autopilot=replace(READY_AUTOPILOT, completed_route_id="m3_water_source")))
        output = fsm.step(
            snapshot(
                mechanism=replace(
                    READY_MECHANISM,
                    water_collection_complete=True,
                    water_volume_l=1.8,
                )
            )
        )
        self.assertEqual(output.state, MissionState.M3_TRANSIT_DROP_TARGET)

        fsm.step(snapshot(autopilot=replace(READY_AUTOPILOT, completed_route_id="m3_water_target")))
        fsm.step(snapshot(vision=replace(READY_VISION, water_target_aligned=True)))
        output = fsm.step(snapshot(mechanism=replace(READY_MECHANISM, water_release_complete=True)))
        self.assertEqual(output.state, MissionState.M1_SURVEY_AREA_1)
        self.assertIn(3, output.completed_missions)

        vehicle = LocatedObservation("vehicle-1", "67-CCF-M-ING", point(20), 0.9, 3.0)
        fsm.step(snapshot(vision=replace(READY_VISION, vehicle_fixes=(vehicle,))))
        output = fsm.step(
            snapshot(
                autopilot=replace(READY_AUTOPILOT, completed_route_id="m1_area1"),
                mapping=replace(READY_MAPPING, area_1_coverage_complete=True),
            )
        )
        self.assertEqual(output.state, MissionState.M1_SURVEY_AREA_2)

        output = fsm.step(
            snapshot(
                autopilot=replace(READY_AUTOPILOT, completed_route_id="m1_area2"),
                mapping=replace(READY_MAPPING, area_2_coverage_complete=True),
            )
        )
        self.assertEqual(output.state, MissionState.M2_SEARCH_HOTSPOTS)
        self.assertIn(1, output.completed_missions)

        hotspots = (
            LocatedObservation("hot-1", "hotspot", point(30), 0.9, 4.0),
            LocatedObservation("hot-2", "hotspot", point(31), 0.8, 5.0),
        )
        output = fsm.step(snapshot(vision=replace(READY_VISION, hotspot_fixes=hotspots)))
        self.assertEqual(output.state, MissionState.M4_SEARCH_DEADMAN)
        self.assertIn(2, output.completed_missions)

        deadman = point(40, altitude=10.0)
        output = fsm.step(
            snapshot(
                sound=SoundInput(
                    ready=True,
                    deadman_detected=True,
                    frequency_hz=2800.0,
                    confidence=0.9,
                    target_fix=deadman,
                    target_position_error_m=2.0,
                )
            )
        )
        self.assertEqual(output.state, MissionState.M4_APPROACH_DEADMAN)
        self.assertEqual(output.route_id, "m4_deadman_approach")

        output = fsm.step(snapshot(autopilot=replace(READY_AUTOPILOT, completed_route_id="m4_deadman_approach")))
        self.assertEqual(output.state, MissionState.M4_DESCEND_FOR_DROP)
        self.assertEqual(output.vision_mode, VisionMode.FIRST_AID_DROP_ALIGNMENT)

        output = fsm.step(
            snapshot(
                autopilot=replace(READY_AUTOPILOT, altitude_agl_m=1.5),
                vision=replace(READY_VISION, drop_horizontal_error_m=0.4),
                mechanism=replace(READY_MECHANISM, first_aid_armed=True),
            )
        )
        self.assertEqual(output.state, MissionState.M4_RELEASE_FIRST_AID)

        output = fsm.step(snapshot(mechanism=replace(READY_MECHANISM, first_aid_release_complete=True)))
        self.assertEqual(output.state, MissionState.RETURN_HOME)
        self.assertEqual(output.completed_missions, (1, 2, 3, 4))

        output = fsm.step(
            snapshot(
                autopilot=replace(READY_AUTOPILOT, completed_route_id="home"),
                vision=replace(READY_VISION, landing_marker_visible=True),
            )
        )
        self.assertEqual(output.state, MissionState.PRECISION_LANDING)
        self.assertEqual(output.flight_command, FlightCommand.PRECISION_LAND)

        output = fsm.step(snapshot(now_s=100.0, autopilot=replace(READY_AUTOPILOT, landed=True)))
        self.assertEqual(output.state, MissionState.POST_FLIGHT_EXPORT)
        self.assertEqual(output.report_deadline_remaining_s, 300.0)

        output = fsm.step(
            snapshot(
                now_s=200.0,
                operator=OperatorInput(post_flight_export_complete=True),
                mapping=replace(READY_MAPPING, map_export_ready=True, map_artifact_uri="map.tif"),
            )
        )
        self.assertEqual(output.state, MissionState.COMPLETE)
        self.assertEqual(output.accepted_vehicle_count, 1)
        self.assertEqual(output.accepted_hotspot_count, 2)
        self.assertEqual(fsm.result_payload()["mission_3"]["released_water_volume_l"], 1.8)
        self.assertEqual(fsm.result_payload()["mission_4"]["horizontal_error_m"], 0.4)


class GuardAndSafetyTests(unittest.TestCase):
    def test_preflight_is_blocked_until_every_team_is_ready(self):
        fsm = OutdoorMissionFSM(plan())
        output = fsm.step(snapshot(operator=OperatorInput(start=True), sound=SoundInput(ready=False)))
        self.assertEqual(output.state, MissionState.PRE_FLIGHT)
        self.assertIn("sound localisation", output.status)

    def test_invalid_gps_fix_is_not_scoring_valid(self):
        fsm = OutdoorMissionFSM(plan(include_area2=False))
        fsm.state = MissionState.M2_SEARCH_HOTSPOTS
        invalid = LocatedObservation("hot-1", "hotspot", point(1), 0.9, 5.1)
        output = fsm.step(snapshot(vision=replace(READY_VISION, hotspot_fixes=(invalid,))))
        self.assertEqual(output.accepted_hotspot_count, 0)
        self.assertEqual(output.state, MissionState.M2_SEARCH_HOTSPOTS)

    def test_low_battery_forces_return(self):
        fsm = OutdoorMissionFSM(plan())
        fsm.state = MissionState.M1_SURVEY_AREA_1
        output = fsm.step(snapshot(autopilot=replace(READY_AUTOPILOT, battery_percent=20.0)))
        self.assertEqual(output.state, MissionState.RETURN_HOME)
        self.assertEqual(output.route_id, "home")

    def test_geofence_violation_requests_emergency_land(self):
        fsm = OutdoorMissionFSM(plan())
        fsm.state = MissionState.M2_SEARCH_HOTSPOTS
        output = fsm.step(snapshot(autopilot=replace(READY_AUTOPILOT, inside_geofence=False)))
        self.assertEqual(output.state, MissionState.EMERGENCY_LAND)
        self.assertEqual(output.flight_command, FlightCommand.EMERGENCY_LAND)

    def test_precision_landing_holds_until_id0_is_visible(self):
        fsm = OutdoorMissionFSM(plan())
        fsm.state = MissionState.PRECISION_LANDING
        airborne_autopilot = replace(READY_AUTOPILOT, airborne=True, landed=False)
        output = fsm.step(snapshot(autopilot=airborne_autopilot, vision=READY_VISION))
        self.assertEqual(output.flight_command, FlightCommand.HOLD_POSITION)
        output = fsm.step(
            snapshot(
                autopilot=airborne_autopilot,
                vision=replace(READY_VISION, landing_marker_visible=True),
            )
        )
        self.assertEqual(output.flight_command, FlightCommand.PRECISION_LAND)

    def test_fixed_wing_landing_does_not_wait_for_aruco(self):
        fsm = OutdoorMissionFSM(plan())
        fsm.state = MissionState.PRECISION_LANDING
        output = fsm.step(
            snapshot(
                autopilot=replace(
                    READY_AUTOPILOT,
                    airborne=True,
                    landed=False,
                    airframe_type="fixed-wing",
                )
            )
        )
        self.assertEqual(output.flight_command, FlightCommand.PRECISION_LAND)
        self.assertEqual(output.vision_mode, VisionMode.IDLE)

    def test_time_margin_forces_return(self):
        fsm = OutdoorMissionFSM(plan())
        fsm.started_at_s = 0.0
        fsm.state = MissionState.M2_SEARCH_HOTSPOTS
        output = fsm.step(snapshot(now_s=1620.0))
        self.assertEqual(output.state, MissionState.RETURN_HOME)

    def test_lawnmower_route_is_deterministic_and_alternating(self):
        south_west = GeoPoint(48.0, 7.0, 30.0)
        north_west = GeoPoint(48.001, 7.0, 30.0)
        south_east = GeoPoint(48.0, 7.001, 30.0)
        north_east = GeoPoint(48.001, 7.001, 30.0)
        route = generate_lawnmower_route(
            south_west,
            north_west,
            south_east,
            north_east,
            lane_spacing_m=50.0,
            altitude_agl_m=30.0,
            prefix="map1",
        )
        self.assertGreaterEqual(len(route), 6)
        self.assertEqual(route[0].position.longitude_deg, 7.0)
        self.assertEqual(route[1].position.longitude_deg, 7.001)
        self.assertEqual(route[2].position.longitude_deg, 7.001)
        self.assertEqual(route[3].position.longitude_deg, 7.0)


if __name__ == "__main__":
    unittest.main()
