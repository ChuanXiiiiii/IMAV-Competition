# IMAV 2026 Outdoor M1/M4 FSM - Integration Contract

Owner: Ziyan - Path Planning and State Machine  
Rule source: `Rulebook_IMAV2026_V4-1.pdf`, especially sections 2.2 and 5.1-5.4  
Code: `imav2026_outdoor_fsm.py` and `simple_waypoint_planner.py`

## 1. Execution profiles

The FSM uses runtime-selected mission plugins. No code change is needed to move between testing and
the final combined run.

| Profile | `enabled_missions` / CLI | Sequence | Required teams at pre-flight |
| --- | --- | --- | --- |
| M1 test | `[1]` / `--missions 1` | Takeoff -> M1 -> return -> land -> export | Autopilot, Vision/Localisation, Mapping |
| M4 test | `[4]` / `--missions 4` | Takeoff -> M4 -> return -> land | Autopilot, Vision/Localisation, Sound, Mechanism |
| Combined | `[1,4]` / `--missions 1 4` | Takeoff -> M1 -> M4 -> return -> land -> export | All above |

Only enabled plugins participate in pre-flight readiness. For example, an unavailable payload
mechanism cannot block an M1-only mapping test.

## 2. V4-1 requirements represented in the FSM

- The competition slot is 30 minutes, with a configurable default 3-minute return reserve.
- A mission becomes successfully validated only after the drone safely returns to its designated
  landing area. Therefore:
  - `accomplished_missions` means the task action has finished in the air;
  - `validated_missions` remains empty until `autopilot.landed=true`.
- M1 Area 1 is 440 x 280 m; Area 2 adds 440 x 320 m.
- An M1 vehicle record is accepted only when its estimated GPS error is at most 5 m.
- M1 results should be exported no later than 5 minutes after landing for the highest submission score.
- M4 uses a 100 x 120 x 40 mm, 200 g first-aid kit attached by its 8 mm zipper pull. The mechanism
  team must assert `first_aid_kit_verified=true` before an M4 profile can start.
- M4 has three mannequins within 25 m of the supplied search point. The correct mannequin has a
  2.6-3.0 kHz alarm (95 dB at 3 m) and two ultra-bright LEDs.
- M4 release altitude must be strictly below 2 m. The FSM does not accept exactly 2.0 m.
- M4 scoring distance is measured from the nearest mannequin's navel; values are bounded from 50 to
  300 cm. The default alignment goal is 0.5 m.
- Multicopter/VTOL precision landing uses the 80 x 80 cm, 5x5 ArUco ID 0 target. Landing zones are
  1 x 1 m precision / 3 x 3 m zone for multicopters and 2 x 2 m / 5 x 5 m for VTOL.
- The flight controller must independently enforce the 80 m AGL limit, geofence and kill switch.

## 3. High-level flows

```text
M1-only:
PRE_FLIGHT -> TAKEOFF -> M1_SURVEY_AREA_1 -> [M1_SURVEY_AREA_2]
-> RETURN_HOME -> PRECISION_LANDING -> POST_FLIGHT_EXPORT -> COMPLETE

M4-only:
PRE_FLIGHT -> TAKEOFF -> M4_SEARCH -> M4_APPROACH -> M4_DESCEND
-> M4_RELEASE -> RETURN_HOME -> PRECISION_LANDING -> COMPLETE

Combined:
PRE_FLIGHT -> TAKEOFF -> M1 states -> M4 states -> RETURN_HOME
-> PRECISION_LANDING -> POST_FLIGHT_EXPORT -> COMPLETE
```

At any active state:

- low battery, return-time margin, mechanism fault or `force_return` -> `RETURN_HOME`;
- geofence violation or operator abort -> `EMERGENCY_LAND`.

## 4. Input ports

The middleware should send one complete `FSMInput` snapshot at 5-10 Hz. Coordinates are WGS84
decimal degrees; altitude is metres AGL; errors/distances are metres; time is monotonic seconds.

### OperatorInput

| Field | Meaning |
| --- | --- |
| `start` | Start enabled profile after judge/team authorisation |
| `abort` | Request emergency landing |
| `force_return` | Stop testing and return home safely |
| `post_flight_export_complete` | M1 map and vehicle table have been exported/submitted |

### AutopilotInput

| Field | Meaning |
| --- | --- |
| `preflight_ok`, `gps_ok` | Required navigation/flight readiness |
| `airborne`, `landed` | Authoritative flight state; landing validates accomplished missions |
| `inside_geofence` | False requests emergency landing |
| `battery_percent` | At/below configured reserve requests return |
| `altitude_agl_m` | M4 release gate requires `<2.0` |
| `completed_route_id` | Exact acknowledgement of the currently commanded route |
| `airframe_type` | `multicopter`, `VTOL`, or `fixed-wing` landing behaviour |

### VisionInput - Vaishnavi and Ilias

| Field | Meaning |
| --- | --- |
| `detection_ready` | Detection pipeline healthy |
| `localisation_and_calibration_ready` | Camera/GPS calibration and localisation healthy |
| `vehicle_fixes` | Stable-ID vehicle observations with label, GPS, confidence and position error |
| `deadman_led_detected`, `deadman_target_fix` | Correct M4 mannequin located from the two LEDs |
| `mannequin_target_fix` | A mannequin candidate when dead-man identity is unconfirmed; allows base M4 score |
| `drop_horizontal_error_m` | Horizontal error relative to the mannequin/navel target |
| `landing_marker_visible` | ArUco 5x5 ID 0 acquired for multicopter/VTOL landing |

Valid M1 labels are firefighter `CCF` or `VLTT` plus the three-letter brigade, or military `VT4`,
`GBC 180`, or `VBL`. `LocatedObservation.observation_id` must remain stable across frames so the FSM
does not count one vehicle repeatedly.

### MappingInput - Kory

| Field | Meaning |
| --- | --- |
| `ready` | Geo-referenced mapping pipeline ready |
| `area_1_coverage_complete`, `area_2_coverage_complete` | Usable area coverage confirmed |
| `map_export_ready` | Final map can be exported after landing |
| `map_artifact_uri` | File path or URI of the final map |

The FSM leaves a mapping area only when both the waypoint route and mapping coverage are complete.

### SoundInput - Herish and Tarun

| Field | Meaning |
| --- | --- |
| `ready` | Sound localisation pipeline ready |
| `deadman_detected` | Candidate alarm detected |
| `frequency_hz` | Must be 2600-3000 Hz |
| `confidence` | FSM acceptance threshold is at least 0.5 |
| `target_fix` | WGS84 location of the alarm |
| `target_position_error_m` | Must be at most 5 m for automatic approach |

### MechanismInput - David and Yingqi

| Field | Meaning |
| --- | --- |
| `ready`, `fault` | Controller health |
| `first_aid_loaded` | Kit physically present |
| `first_aid_kit_verified` | V4-1 size/weight/8 mm attachment requirements checked |
| `first_aid_armed` | Release mechanism armed and feedback healthy |
| `first_aid_release_complete` | Physical release confirmed |

## 5. Output ports

Every `step()` call returns the desired current `FSMOutput`; commands are levels, not one-frame pulses.

| Field | Consumer/purpose |
| --- | --- |
| `state`, `profile_missions`, `active_mission` | All modules, UI and logging |
| `flight_command` | Autopilot: hold, takeoff, follow, descend, precision-land or emergency-land |
| `route_id`, `route`, `route_repeat` | Autopilot: ordered WGS84 waypoints and exact acknowledgement ID |
| `target_altitude_agl_m` | Autopilot takeoff/M4 descent setpoint |
| `vision_mode` | Select M1 mapping, M4 search/alignment, or landing detector |
| `sound_mode` | Enable the M4 2.6-3.0 kHz pipeline only when required |
| `mechanism_command` | `SAFE`, `ARM_FIRST_AID`, or `RELEASE_FIRST_AID` |
| `export_results` | Mapping/reporting module should immediately export M1 results |
| `accomplished_missions` | Air task actions completed but not necessarily rule-validated |
| `validated_missions` | Accomplished missions confirmed by safe landing |
| `accepted_vehicle_count` | Deduplicated, scoring-valid M1 records |
| `report_deadline_remaining_s` | M1 300-second post-landing export countdown |
| `slot_deadline_remaining_s` | Whole-profile 30-minute countdown |
| `status` | Human-readable transition/wait reason |

## 6. State guards

| State | Transition guard | Main outputs |
| --- | --- | --- |
| `PRE_FLIGHT` | Start + enabled-plugin readiness | HOLD, payload SAFE |
| `TAKEOFF` | `airborne=true` | TAKEOFF; then first enabled plugin |
| `M1_SURVEY_AREA_1` | `m1_area1` complete + Area 1 coverage | M1 lawnmower route and vehicle mode |
| `M1_SURVEY_AREA_2` | `m1_area2` complete + Area 2 coverage | Extended M1 route and vehicle mode |
| `M4_SEARCH` | Valid sound/LED fix, or an unconfirmed mannequin fix | Repeating search route, sound + vision, kit armed |
| `M4_APPROACH` | Dynamic `m4_target_approach` complete | Waypoint above selected mannequin |
| `M4_DESCEND` | AGL `<2 m`, alignment within goal, mechanism armed | Controlled descent and alignment |
| `M4_RELEASE` | Physical release confirmation | Hold and command release |
| `RETURN_HOME` | `home` route complete | Safe home route, mechanism SAFE |
| `PRECISION_LANDING` | `landed=true` | ID 0 landing for multicopter/VTOL; then validate missions |
| `POST_FLIGHT_EXPORT` | Map ready + export confirmation | DISARMED and 300 s countdown |
| `COMPLETE` | Terminal | DISARMED |
| `EMERGENCY_LAND` | Terminal safety request | Emergency-land and mechanism SAFE |

## 7. Waypoint configuration

Exact zone and landing GPS coordinates are supplied on competition day. Copy
`config/mission_plan.template.json`, fill real coordinates, plot them against the geofence, and run:

```bash
python imav2026_outdoor_fsm.py --plan config/mission_plan.day_of.json --missions 1 --validate-plan
python imav2026_outdoor_fsm.py --plan config/mission_plan.day_of.json --missions 4 --validate-plan
python imav2026_outdoor_fsm.py --plan config/mission_plan.day_of.json --missions 1 4 --validate-plan
```

Required routes are profile-specific:

- M1: `m1_area1`, optionally `m1_area2`, and `home`;
- M4: `m4_deadman_search` and `home`;
- combined: all of the above.

Use `generate_lawnmower_route()` for deterministic area coverage. Choose M1 lane spacing from the
calibrated camera footprint/overlap and M4 lane spacing from measured sound/vision detection range.

## 8. Integration skeleton

```python
plan = load_plan_json("config/mission_plan.day_of.json", mission_override=[1, 4])
fsm = OutdoorMissionFSM(plan)

while middleware_is_running:
    inputs = collect_complete_fsm_input_snapshot()
    output = fsm.step(inputs)
    publish_autopilot(output.flight_command, output.route_id, output.route)
    publish_vision_mode(output.vision_mode)
    publish_sound_mode(output.sound_mode)
    publish_mechanism_command(output.mechanism_command)
```

The precision-landing and flight controllers retain low-level control authority. The mechanism must
also independently inhibit release at or above 2 m so a software/communications fault cannot produce
an invalid or unsafe drop.
