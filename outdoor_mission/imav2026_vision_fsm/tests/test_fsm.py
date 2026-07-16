import importlib.util
import sys
import types
import unittest
from pathlib import Path


if "cv2" not in sys.modules:
    sys.modules["cv2"] = types.ModuleType("cv2")

MODULE_PATH = Path(__file__).parents[1] / "imav2026_outdoor_fsm.py"
SPEC = importlib.util.spec_from_file_location("imav2026_outdoor_fsm", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

MissionEvent = MODULE.MissionEvent
MissionState = MODULE.MissionState
OutdoorMissionFSM = MODULE.OutdoorMissionFSM


class OutdoorMissionFSMTests(unittest.TestCase):
    def update(self, fsm, event=None, value=None, now=0.0, marker=None):
        centers = {} if marker is None else {0: marker}
        fsm.update(set(centers), centers, (320, 240), event, value, now)

    def land(self, fsm):
        self.update(fsm, now=10.0, marker=(320, 240))
        self.update(fsm, now=10.1, marker=(320, 240))
        self.update(fsm, now=15.2, marker=(320, 240))

    def test_mission_1_requires_area_1_and_finishes_after_report(self):
        fsm = OutdoorMissionFSM(1)
        self.update(fsm, MissionEvent.TAKEOFF)
        self.update(fsm, MissionEvent.MAPPING_FINISHED)
        self.assertEqual(fsm.state, MissionState.STATE_MISSION_1_MAPPING)
        self.update(fsm, MissionEvent.AREA_1_MAPPED)
        self.update(fsm, MissionEvent.VEHICLE_FOUND)
        self.update(fsm, MissionEvent.MAPPING_FINISHED)
        self.land(fsm)
        self.assertEqual(fsm.state, MissionState.STATE_REPORT)
        self.update(fsm, MissionEvent.REPORT_SUBMITTED, now=100.0)
        self.assertTrue(fsm.report_on_time)
        self.assertEqual(fsm.state, MissionState.STATE_COMPLETED)

    def test_mission_2_returns_after_two_hotspots(self):
        fsm = OutdoorMissionFSM(2)
        self.update(fsm, MissionEvent.TAKEOFF)
        self.update(fsm, MissionEvent.HOTSPOT_FOUND)
        self.assertEqual(fsm.state, MissionState.STATE_MISSION_2_FIRE_SEARCH)
        self.update(fsm, MissionEvent.HOTSPOT_FOUND)
        self.assertEqual(fsm.state, MissionState.STATE_RETURN)

    def test_mission_3_collect_drop_land(self):
        fsm = OutdoorMissionFSM(3)
        self.update(fsm, MissionEvent.TAKEOFF)
        self.update(fsm, MissionEvent.WATER_COLLECTED)
        self.assertEqual(fsm.state, MissionState.STATE_MISSION_3_DROP_WATER)
        self.update(fsm, MissionEvent.WATER_DROPPED)
        self.land(fsm)
        self.assertEqual(fsm.state, MissionState.STATE_COMPLETED)

    def test_mission_4_rejects_drop_above_two_metres(self):
        fsm = OutdoorMissionFSM(4)
        self.update(fsm, MissionEvent.TAKEOFF)
        self.update(fsm, MissionEvent.DEADMAN_FOUND)
        self.update(fsm, MissionEvent.PACKAGE_DROPPED, value=2.1)
        self.assertEqual(fsm.state, MissionState.STATE_MISSION_4_APPROACH)
        self.update(fsm, MissionEvent.PACKAGE_DROPPED, value=2.0)
        self.assertEqual(fsm.state, MissionState.STATE_RETURN)

    def test_landing_hold_resets_when_marker_is_lost(self):
        fsm = OutdoorMissionFSM(3)
        self.update(fsm, MissionEvent.TAKEOFF)
        self.update(fsm, MissionEvent.WATER_COLLECTED)
        self.update(fsm, MissionEvent.WATER_DROPPED)
        self.update(fsm, now=1.0, marker=(320, 240))
        self.update(fsm, now=1.1, marker=(320, 240))
        self.update(fsm, now=3.0)
        self.assertEqual(fsm.state, MissionState.STATE_RETURN)
        self.assertIsNone(fsm.landing_hold_start)


if __name__ == "__main__":
    unittest.main()
