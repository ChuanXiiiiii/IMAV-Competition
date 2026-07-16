import argparse
import time
from enum import Enum
from typing import Dict, Optional, Set, Tuple

import cv2


class MissionState(Enum):
    STATE_IDLE = "STATE_IDLE"
    STATE_MISSION_1_MAPPING = "STATE_MISSION_1_MAPPING"
    STATE_MISSION_2_FIRE_SEARCH = "STATE_MISSION_2_FIRE_SEARCH"
    STATE_MISSION_3_COLLECT_WATER = "STATE_MISSION_3_COLLECT_WATER"
    STATE_MISSION_3_DROP_WATER = "STATE_MISSION_3_DROP_WATER"
    STATE_MISSION_4_SEARCH = "STATE_MISSION_4_SEARCH"
    STATE_MISSION_4_APPROACH = "STATE_MISSION_4_APPROACH"
    STATE_RETURN = "STATE_RETURN"
    STATE_PRECISION_LANDING = "STATE_PRECISION_LANDING"
    STATE_REPORT = "STATE_REPORT"
    STATE_COMPLETED = "STATE_COMPLETED"
    STATE_ABORTED = "STATE_ABORTED"


class MissionEvent(Enum):
    TAKEOFF = "TAKEOFF"
    AREA_1_MAPPED = "AREA_1_MAPPED"
    AREA_2_MAPPED = "AREA_2_MAPPED"
    VEHICLE_FOUND = "VEHICLE_FOUND"
    MAPPING_FINISHED = "MAPPING_FINISHED"
    HOTSPOT_FOUND = "HOTSPOT_FOUND"
    WATER_COLLECTED = "WATER_COLLECTED"
    WATER_DROPPED = "WATER_DROPPED"
    DEADMAN_FOUND = "DEADMAN_FOUND"
    PACKAGE_DROPPED = "PACKAGE_DROPPED"
    REPORT_SUBMITTED = "REPORT_SUBMITTED"
    ABORT = "ABORT"


class OutdoorMissionFSM:
    """Rulebook-driven FSM for one IMAV 2026 outdoor mission flight."""

    def __init__(self, mission_number: int) -> None:
        if mission_number not in (1, 2, 3, 4):
            raise ValueError("mission_number must be one of 1, 2, 3, or 4")

        self.mission_number = mission_number
        self.state = MissionState.STATE_IDLE

        self.area_1_mapped = False
        self.area_2_mapped = False
        self.vehicles_found = 0
        self.hotspots_found = 0
        self.deadman_found = False
        self.drop_altitude_m: Optional[float] = None

        self.center_tolerance_px = 30
        self.hold_required_seconds = 5.0
        self.landing_hold_start: Optional[float] = None
        self.latest_offset_xy = (0, 0)

        self.landed_at: Optional[float] = None
        self.report_submitted_at: Optional[float] = None
        self.report_on_time: Optional[bool] = None

        print(f"Waiting for Mission {mission_number} takeoff...")

    def transition_to(self, new_state: MissionState) -> None:
        if self.state == new_state:
            return

        print(f"[FSM] {self.state.value} -> {new_state.value}")
        self.state = new_state

        if new_state == MissionState.STATE_RETURN:
            print("[RETURN] Returning to the landing zone; searching for ArUco ID 0.")

        if new_state != MissionState.STATE_PRECISION_LANDING:
            self.landing_hold_start = None

    def _start_selected_mission(self) -> None:
        first_states = {
            1: MissionState.STATE_MISSION_1_MAPPING,
            2: MissionState.STATE_MISSION_2_FIRE_SEARCH,
            3: MissionState.STATE_MISSION_3_COLLECT_WATER,
            4: MissionState.STATE_MISSION_4_SEARCH,
        }
        self.transition_to(first_states[self.mission_number])

    def _finish_landing(self, now: float) -> None:
        self.landed_at = now
        print("[LANDING] ID 0 centered for 5 s; precision landing completed.")
        if self.mission_number in (1, 2):
            self.transition_to(MissionState.STATE_REPORT)
            print("[REPORT] Submit results within 5 minutes of landing (press 's').")
        else:
            self.transition_to(MissionState.STATE_COMPLETED)

    def update(
        self,
        detected_ids: Set[int],
        id_centers: Dict[int, Tuple[int, int]],
        frame_center: Tuple[int, int],
        event: Optional[MissionEvent] = None,
        event_value: Optional[float] = None,
        now: Optional[float] = None,
    ) -> None:
        current_time = time.monotonic() if now is None else now

        if event == MissionEvent.ABORT:
            self.transition_to(MissionState.STATE_ABORTED)
            return

        if self.state == MissionState.STATE_IDLE:
            if event == MissionEvent.TAKEOFF:
                self._start_selected_mission()

        elif self.state == MissionState.STATE_MISSION_1_MAPPING:
            if event == MissionEvent.AREA_1_MAPPED:
                self.area_1_mapped = True
                print("[MISSION_1] Mapping Area 1 completed.")
            elif event == MissionEvent.AREA_2_MAPPED:
                if not self.area_1_mapped:
                    print("[MISSION_1] Map Area 1 before recording Area 2.")
                else:
                    self.area_2_mapped = True
                    print("[MISSION_1] Mapping Area 2 completed.")
            elif event == MissionEvent.VEHICLE_FOUND:
                if self.vehicles_found < 8:
                    self.vehicles_found += 1
                    print(f"[MISSION_1] Vehicle recorded ({self.vehicles_found}/8).")
            elif event == MissionEvent.MAPPING_FINISHED:
                if self.area_1_mapped:
                    self.transition_to(MissionState.STATE_RETURN)
                else:
                    print("[MISSION_1] Cannot finish before Mapping Area 1 is complete.")

        elif self.state == MissionState.STATE_MISSION_2_FIRE_SEARCH:
            if event == MissionEvent.HOTSPOT_FOUND and self.hotspots_found < 2:
                self.hotspots_found += 1
                print(f"[MISSION_2] Hot spot recorded ({self.hotspots_found}/2).")
                if self.hotspots_found == 2:
                    self.transition_to(MissionState.STATE_RETURN)

        elif self.state == MissionState.STATE_MISSION_3_COLLECT_WATER:
            if event == MissionEvent.WATER_COLLECTED:
                self.transition_to(MissionState.STATE_MISSION_3_DROP_WATER)

        elif self.state == MissionState.STATE_MISSION_3_DROP_WATER:
            if event == MissionEvent.WATER_DROPPED:
                self.transition_to(MissionState.STATE_RETURN)

        elif self.state == MissionState.STATE_MISSION_4_SEARCH:
            if event == MissionEvent.DEADMAN_FOUND:
                self.deadman_found = True
                self.transition_to(MissionState.STATE_MISSION_4_APPROACH)

        elif self.state == MissionState.STATE_MISSION_4_APPROACH:
            if event == MissionEvent.PACKAGE_DROPPED:
                if event_value is None:
                    print("[MISSION_4] Drop altitude is required.")
                elif event_value > 2.0:
                    print("[MISSION_4] Drop rejected: altitude must be <= 2.0 m.")
                else:
                    self.drop_altitude_m = event_value
                    self.transition_to(MissionState.STATE_RETURN)

        elif self.state == MissionState.STATE_RETURN:
            if 0 in detected_ids:
                self.transition_to(MissionState.STATE_PRECISION_LANDING)

        elif self.state == MissionState.STATE_PRECISION_LANDING:
            if 0 not in id_centers:
                self.landing_hold_start = None
                self.transition_to(MissionState.STATE_RETURN)
                return

            marker_x, marker_y = id_centers[0]
            center_x, center_y = frame_center
            dx = marker_x - center_x
            dy = marker_y - center_y
            self.latest_offset_xy = (dx, dy)
            aligned = abs(dx) <= self.center_tolerance_px and abs(dy) <= self.center_tolerance_px

            if not aligned:
                self.landing_hold_start = None
                return

            if self.landing_hold_start is None:
                self.landing_hold_start = current_time
            elif current_time - self.landing_hold_start >= self.hold_required_seconds:
                self._finish_landing(current_time)

        elif self.state == MissionState.STATE_REPORT:
            if event == MissionEvent.REPORT_SUBMITTED:
                self.report_submitted_at = current_time
                assert self.landed_at is not None
                self.report_on_time = current_time - self.landed_at <= 300.0
                status = "within" if self.report_on_time else "after"
                print(f"[REPORT] Results submitted {status} the 5-minute scoring window.")
                self.transition_to(MissionState.STATE_COMPLETED)

    def get_overlay_lines(self) -> list[str]:
        lines = [f"Mission: {self.mission_number}", f"State: {self.state.value}"]

        if self.state == MissionState.STATE_MISSION_1_MAPPING:
            lines.extend(
                [
                    f"Area 1 mapped: {self.area_1_mapped}",
                    f"Area 2 mapped: {self.area_2_mapped}",
                    f"Vehicles: {self.vehicles_found}/8",
                ]
            )
        elif self.state == MissionState.STATE_MISSION_2_FIRE_SEARCH:
            lines.append(f"Hot spots: {self.hotspots_found}/2")
        elif self.state == MissionState.STATE_MISSION_4_SEARCH:
            lines.append("Searching for 2.6-3.0 kHz alarm / bright LEDs")
        elif self.state == MissionState.STATE_MISSION_4_APPROACH:
            lines.append("Drop altitude must be <= 2.0 m")
        elif self.state == MissionState.STATE_PRECISION_LANDING:
            dx, dy = self.latest_offset_xy
            lines.append(f"ID0 offset: x={dx}, y={dy} px")
            held = 0.0 if self.landing_hold_start is None else time.monotonic() - self.landing_hold_start
            lines.append(f"Centered hold: {min(held, 5.0):.1f} / 5.0 s")
        elif self.state == MissionState.STATE_REPORT and self.landed_at is not None:
            elapsed = time.monotonic() - self.landed_at
            lines.append(f"Report timer: {elapsed:.1f} / 300.0 s")

        return lines


def create_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters) if hasattr(cv2.aruco, "ArucoDetector") else None
    return aruco_dict, parameters, detector


def detect_aruco_markers(frame, aruco_dict, parameters, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if detector is not None:
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)


def collect_marker_centers(corners, ids) -> Dict[int, Tuple[int, int]]:
    centers: Dict[int, Tuple[int, int]] = {}
    if ids is None:
        return centers
    for marker_id, marker_corners in zip(ids.flatten(), corners):
        points = marker_corners[0]
        centers[int(marker_id)] = (int(points[:, 0].mean()), int(points[:, 1].mean()))
    return centers


def draw_overlay(frame, lines: list[str]) -> None:
    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (10, 28 + index * 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (40, 255, 40),
            2,
            cv2.LINE_AA,
        )


def event_from_key(key: int, mission_number: int) -> Optional[MissionEvent]:
    common = {ord("t"): MissionEvent.TAKEOFF, ord("x"): MissionEvent.ABORT}
    mission_keys = {
        1: {
            ord("1"): MissionEvent.AREA_1_MAPPED,
            ord("2"): MissionEvent.AREA_2_MAPPED,
            ord("v"): MissionEvent.VEHICLE_FOUND,
            ord("f"): MissionEvent.MAPPING_FINISHED,
            ord("s"): MissionEvent.REPORT_SUBMITTED,
        },
        2: {ord("h"): MissionEvent.HOTSPOT_FOUND, ord("s"): MissionEvent.REPORT_SUBMITTED},
        3: {ord("c"): MissionEvent.WATER_COLLECTED, ord("d"): MissionEvent.WATER_DROPPED},
        4: {ord("b"): MissionEvent.DEADMAN_FOUND, ord("p"): MissionEvent.PACKAGE_DROPPED},
    }
    return common.get(key) or mission_keys[mission_number].get(key)


def run(camera_index: int, mission_number: int, drop_altitude: float) -> None:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera index {camera_index}")
        return

    aruco_dict, parameters, detector = create_aruco_detector()
    fsm = OutdoorMissionFSM(mission_number)
    print("[INFO] Press 'q' to quit, 't' to take off, or 'x' to abort.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Failed to read frame from camera.")
                break

            frame_h, frame_w = frame.shape[:2]
            frame_center = (frame_w // 2, frame_h // 2)
            corners, ids, _ = detect_aruco_markers(frame, aruco_dict, parameters, detector)
            id_centers = collect_marker_centers(corners, ids)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawMarker(frame, frame_center, (255, 200, 0), cv2.MARKER_CROSS, 22, 2)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            event = event_from_key(key, mission_number)
            value = drop_altitude if event == MissionEvent.PACKAGE_DROPPED else None
            fsm.update(set(id_centers), id_centers, frame_center, event, value)

            draw_overlay(frame, fsm.get_overlay_lines())
            cv2.imshow("IMAV 2026 Outdoor Mission FSM", frame)

            if fsm.state in (MissionState.STATE_COMPLETED, MissionState.STATE_ABORTED):
                print(f"[FSM] {fsm.state.value} reached, exiting.")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(description="IMAV 2026 outdoor mission FSM simulation")
    parser.add_argument("--mission", type=int, choices=(1, 2, 3, 4), required=True)
    parser.add_argument("--camera", type=int, default=0, help="Camera index, default: 0")
    parser.add_argument(
        "--drop-altitude",
        type=float,
        default=1.5,
        help="Simulated Mission 4 package drop altitude in metres, default: 1.5",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(args.camera, args.mission, args.drop_altitude)
