import argparse
import time
from enum import Enum
from typing import Dict, Optional, Set, Tuple

import cv2


class MissionState(Enum):
    STATE_IDLE = "STATE_IDLE"
    STATE_MISSION_1_3 = "STATE_MISSION_1_3"
    STATE_MISSION_2 = "STATE_MISSION_2"
    # 为避免命名冲突，这里把"寻找4/5/6"阶段单独命名为 SEARCH。
    STATE_MISSION_4_SEARCH = "STATE_MISSION_4_SEARCH"
    STATE_MISSION_4 = "STATE_MISSION_4"
    STATE_RETURN = "STATE_RETURN"
    STATE_LANDED = "STATE_LANDED"


class VisionMissionFSM:
    def __init__(self) -> None:
        self.state = MissionState.STATE_IDLE
        self.mission2_completed = False
        self.route_found = False

        self.center_tolerance_px = 30
        self.hold_required_seconds = 5.0
        self.id4_hold_start: Optional[float] = None
        self.latest_offset_xy = (0, 0)

        print("Waiting for Takeoff...")

    def transition_to(self, new_state: MissionState) -> None:
        if self.state == new_state:
            return

        print(f"[FSM] {self.state.value} -> {new_state.value}")
        self.state = new_state

        if new_state == MissionState.STATE_MISSION_2:
            print("[MISSION_2] Press Enter to simulate mission completion.")

        if new_state == MissionState.STATE_RETURN:
            self.route_found = False
            print("[RETURN] Searching ArUco ID 3 to find route...")

        if new_state != MissionState.STATE_MISSION_4:
            self.id4_hold_start = None

    def update(
        self,
        detected_ids: Set[int],
        id_centers: Dict[int, Tuple[int, int]],
        frame_center: Tuple[int, int],
        key: int,
    ) -> None:
        if self.state == MissionState.STATE_IDLE:
            if 0 in detected_ids:
                self.transition_to(MissionState.STATE_MISSION_1_3)

        elif self.state == MissionState.STATE_MISSION_1_3:
            if 2 in detected_ids:
                self.mission2_completed = False
                self.transition_to(MissionState.STATE_MISSION_2)

        elif self.state == MissionState.STATE_MISSION_2:
            if key in (10, 13):
                if not self.mission2_completed:
                    self.mission2_completed = True
                    print("[MISSION_2] Mission marked as completed (Enter pressed).")

            if self.mission2_completed and 2 in detected_ids:
                self.transition_to(MissionState.STATE_MISSION_4_SEARCH)

        elif self.state == MissionState.STATE_MISSION_4_SEARCH:
            if 4 in detected_ids and 5 in detected_ids and 6 in detected_ids:
                self.transition_to(MissionState.STATE_MISSION_4)

        elif self.state == MissionState.STATE_MISSION_4:
            if 4 in id_centers:
                id4_x, id4_y = id_centers[4]
                center_x, center_y = frame_center
                dx = id4_x - center_x
                dy = id4_y - center_y
                self.latest_offset_xy = (dx, dy)

                aligned = abs(dx) <= self.center_tolerance_px and abs(dy) <= self.center_tolerance_px
                if aligned:
                    if self.id4_hold_start is None:
                        self.id4_hold_start = time.time()
                    held_seconds = time.time() - self.id4_hold_start
                    if held_seconds >= self.hold_required_seconds:
                        print("[MISSION_4] ID 4 centered for 5s, grasp success simulated.")
                        self.transition_to(MissionState.STATE_RETURN)
                else:
                    self.id4_hold_start = None
            else:
                self.id4_hold_start = None

        elif self.state == MissionState.STATE_RETURN:
            if not self.route_found and 3 in detected_ids:
                self.route_found = True
                print("[RETURN] Route Found")

            if self.route_found and 1 in detected_ids:
                self.transition_to(MissionState.STATE_LANDED)

    def get_overlay_lines(self, detected_ids: Set[int]) -> list[str]:
        lines = [f"State: {self.state.value}"]

        if self.state == MissionState.STATE_MISSION_2:
            lines.append(f"MISSION_2 completed: {self.mission2_completed}")
            lines.append("Press Enter to set completion TRUE")

        if self.state == MissionState.STATE_MISSION_4_SEARCH:
            for marker_id in (4, 5, 6):
                status = "Detected" if marker_id in detected_ids else "not Detected"
                lines.append(f"ID {marker_id}: {status}")

        if self.state == MissionState.STATE_MISSION_4:
            dx, dy = self.latest_offset_xy
            lines.append(f"ID4 offset: x={dx}, y={dy} px")

            if self.id4_hold_start is None:
                lines.append("Centered hold: 0.0 / 5.0 s")
            else:
                held = time.time() - self.id4_hold_start
                lines.append(f"Centered hold: {min(held, 5.0):.1f} / 5.0 s")

        if self.state == MissionState.STATE_RETURN:
            lines.append("Route status: Route Found" if self.route_found else "Route status: Searching")

        return lines


def create_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()

    detector = None
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    return aruco_dict, parameters, detector


def detect_aruco_markers(frame, aruco_dict, parameters, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if detector is not None:
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids, rejected


def collect_marker_centers(corners, ids) -> Dict[int, Tuple[int, int]]:
    centers: Dict[int, Tuple[int, int]] = {}
    if ids is None:
        return centers

    for marker_id, marker_corners in zip(ids.flatten(), corners):
        pts = marker_corners[0]
        cx = int(pts[:, 0].mean())
        cy = int(pts[:, 1].mean())
        centers[int(marker_id)] = (cx, cy)
    return centers


def draw_overlay(frame, lines: list[str]) -> None:
    y = 28
    for line in lines:
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (40, 255, 40), 2, cv2.LINE_AA)
        y += 26


def draw_frame_center(frame, frame_center: Tuple[int, int]) -> None:
    cx, cy = frame_center
    cv2.drawMarker(frame, (cx, cy), (255, 200, 0), cv2.MARKER_CROSS, 22, 2)


def run(camera_index: int) -> None:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera index {camera_index}")
        return

    aruco_dict, parameters, detector = create_aruco_detector()
    fsm = VisionMissionFSM()

    print("[INFO] Press 'q' to quit. Press Enter in STATE_MISSION_2 to set completion TRUE.")

    key = -1
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
            detected_ids = set(id_centers.keys())

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for marker_id, center in id_centers.items():
                    cv2.circle(frame, center, 4, (0, 255, 255), -1)
                    cv2.putText(
                        frame,
                        f"ID {marker_id}",
                        (center[0] + 6, center[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

            fsm.update(detected_ids, id_centers, frame_center, key)

            draw_frame_center(frame, frame_center)
            draw_overlay(frame, fsm.get_overlay_lines(detected_ids))

            cv2.imshow("IMAV 2026 Indoor Mission FSM", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] Quit requested by user.")
                break

            if fsm.state == MissionState.STATE_LANDED:
                print("[FSM] STATE_LANDED reached, exiting.")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


def parse_args():
    parser = argparse.ArgumentParser(description="IMAV 2026 indoor mission vision FSM simulation")
    parser.add_argument("--camera", type=int, default=0, help="Camera index, default: 0")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(args.camera)
