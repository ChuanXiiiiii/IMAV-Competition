# IMAV 2026 Outdoor Mission FSM - Team Interface Contract

Owner: Ziyan (Path Planning and State Machine)  
Rule source: `Rulebook_IMAV2026_V2.pdf`, sections 5.2-5.4, pages 22-33  
Implementation: `imav2026_outdoor_fsm.py` and `simple_waypoint_planner.py`

## 1. What this FSM does

The FSM is the macro-level mission orchestrator. It does not run detectors, estimate GPS positions,
localise sound, actuate payload hardware directly, or stabilise the aircraft. It selects the active
mission phase and publishes requests to those modules. The modules return typed status/results, and
the FSM advances only when the relevant completion guard is satisfied.

The default single-flight order is:

```mermaid
flowchart LR
    P[Pre-flight] --> T[Takeoff]
    T --> M3[Mission 3: collect and drop water]
    M3 --> M1[Mission 1: map and identify vehicles]
    M1 --> M2[Mission 2: locate two hot spots]
    M2 --> M4[Mission 4: locate dead-man and drop first aid]
    M4 --> R[Return home]
    R --> L[ArUco ID 0 precision landing]
    L --> E[Export M1 and M2 results within 5 min]
    E --> C[Complete]
```

Why this order: Figure 16 places Mission 3 next to the takeoff/landing zone, the mapping/fire areas
across the main field, and the dead-man area near the far/north-west side. A deterministic loop is
easier to test than online route optimisation. Mission 2 uses one drone as required. Completing
multiple missions in one flight is explicitly allowed, and the landing bonus applies to every mission
completed in that flight.

Trade-off: Mission 2 flight time/energy scoring includes the whole flight, so a dedicated Mission 2
flight could score better on its power factor. The chosen plan follows the team's stated priority:
reliable completion of all mission actions.

## 2. Shared message conventions

- Update rate: FSM `step()` should receive a complete snapshot at 5-10 Hz.
- Coordinates: WGS84 latitude/longitude in decimal degrees.
- Altitude: metres AGL, not MSL.
- Distance/error: metres; sound frequency: Hz; water volume: litres.
- Time: monotonic seconds in `FSMInput.now_s`.
- A route is complete only when Autopilot returns exactly the current `route_id` in
  `completed_route_id`. This prevents a stale completion event from advancing the next state.
- When `route_repeat=true`, Autopilot repeats that route until the FSM changes state; this is used
  for the M2 thermal search and M4 dead-man search.
- Completion/fault fields from hardware should stay latched until the FSM leaves that phase.
- Detection IDs must be stable across frames so duplicate observations are not counted twice.
- Mission 1/2 fixes are accepted for scoring only when confidence is at least `0.5` (team design
  threshold) and the localisation module reports horizontal error at most `5 m` (rulebook limit).

## 3. Team ownership

| Team/member | Data supplied to FSM | Requests received from FSM |
| --- | --- | --- |
| Vaishnavi - Detection | Vehicle classes/IDs, thermal hot spots, water/target alignment, mannequin/LED, ArUco ID 0 | `VisionMode` |
| Ilias - Localisation and Calibration | WGS84 fixes, position-error estimates, calibrated offsets, readiness | `VisionMode`; current route/phase context |
| Kory - Mapping | Area coverage completion, map readiness/export | `MAP_AND_IDENTIFY_VEHICLES`; M1 route |
| Herish and Tarun - Sound Localisation | 2.6-3.0 kHz detection, confidence, WGS84 target fix and error | `SoundMode.DEADMAN_2_6_TO_3_0_KHZ` |
| David and Yingqi - Mechanism | Payload readiness, water volume/collection/release, kit armed/released, faults | `MechanismCommand` |
| Ziyan - Planning/FSM | State, route, flight command, requested perception/sound/mechanism mode, export trigger | All typed input snapshots |
| Autopilot integration | Airborne/landed, route acknowledgement, GPS/geofence, battery, altitude AGL | `FlightCommand`, route and target altitude |

## 4. Input ports

All inputs are grouped in one immutable `FSMInput` snapshot.

### 4.1 OperatorInput

| Field | Type | Meaning |
| --- | --- | --- |
| `start` | bool | One start request after judges/crew authorise flight |
| `abort` | bool | Immediate emergency-land request |
| `force_return` | bool | Stop mission progression and follow the home route |
| `post_flight_export_complete` | bool | M1 map/vehicle table and M2 hotspot table have been exported/submitted |

### 4.2 AutopilotInput

| Field | Type | Meaning |
| --- | --- | --- |
| `preflight_ok` | bool | Arming, navigation and flight checks passed |
| `airborne`, `landed` | bool | Authoritative vehicle states |
| `gps_ok` | bool | GPS solution suitable for route execution |
| `inside_geofence` | bool | False immediately enters `EMERGENCY_LAND` |
| `battery_percent` | float | At/below configured reserve forces `RETURN_HOME` |
| `altitude_agl_m` | float | Used for the Mission 4 release gate (`<= 2.0 m`) |
| `completed_route_id` | optional string | Acknowledges completion of exactly one commanded route |
| `flight_time_s` | float | Takeoff-to-stop time logged for Mission 2 energy scoring |
| `battery_capacity_mah` | optional float | Legible battery capacity used for Mission 2 power factor |
| `battery_max_voltage_v` | optional float | Maximum voltage after organisers recharge the battery |
| `airframe_type` | string | `multicopter`, `fixed-wing`, or `VTOL` for Mission 2 configuration factor |

### 4.3 VisionInput (Detection + Localisation/Calibration)

| Field | Type | Producer | Meaning |
| --- | --- | --- | --- |
| `detection_ready` | bool | Vaishnavi | Detector pipeline healthy |
| `localisation_and_calibration_ready` | bool | Ilias | Camera calibration and GPS projection healthy |
| `vehicle_fixes` | tuple of `LocatedObservation` | Vaishnavi + Ilias | Vehicle ID/type and GPS fix; error must be `<=5 m` |
| `hotspot_fixes` | tuple of `LocatedObservation` | Detection + Ilias | Thermal target GPS fixes; two unique valid fixes required |
| `water_source_aligned` | bool | Vision | Collection system aligned over basin (advisory during transit/collection) |
| `water_target_aligned` | bool | Vision | Aligned over the 1.5 x 1.5 m red target; gates water release |
| `deadman_led_detected` | bool | Vision | Ultra-bright LEDs on the correct mannequin detected |
| `deadman_target_fix` | optional `GeoPoint` | Vision + Ilias | GPS approach target when LEDs identify the dead-man mannequin |
| `drop_horizontal_error_m` | optional float | Vision + Ilias | Kit release point error relative to target navel; design goal `<=0.5 m` |
| `landing_marker_visible` | bool | Vision | ArUco 5x5 ID 0 visible; consumed by the lower-level precision landing controller |

`LocatedObservation` contains: `observation_id`, `label`, `GeoPoint`, `confidence`, and
`position_error_m`. Mission 1 labels should follow the rulebook vocabulary: fire brigade `CCF` or
`VLTT` plus the three-letter brigade, or military `VT4`, `GBC 180`, or `VBL`.

### 4.4 MappingInput

| Field | Type | Meaning |
| --- | --- | --- |
| `ready` | bool | Mapping pipeline can record geo-referenced images |
| `area_1_coverage_complete` | bool | Kory confirms usable coverage of 440 x 280 m Mapping Area 1 |
| `area_2_coverage_complete` | bool | Kory confirms usable coverage of the additional 440 x 320 m area |
| `map_export_ready` | bool | Final map can be exported after landing |
| `map_artifact_uri` | optional string | Path/URI of the final geo-referenced map for the result payload |

Both route completion and mapping coverage must be true before the FSM leaves each mapping area.

### 4.5 SoundInput

| Field | Type | Meaning |
| --- | --- | --- |
| `ready` | bool | Sound pipeline ready before takeoff |
| `deadman_detected` | bool | Candidate MSA Motion Scout alarm detected |
| `frequency_hz` | optional float | Must be within the rulebook range 2600-3000 Hz |
| `confidence` | float | Must be at least 0.5 (team design threshold) |
| `target_fix` | optional `GeoPoint` | Localised target used to build the dynamic approach waypoint |
| `target_position_error_m` | optional float | Must be at most 5 m for automatic approach acceptance |

The alarm is specified as 95 dB at 3 m and is accompanied by two ultra-bright LEDs. Sound and
vision are parallel identification routes; either can produce the final target fix.

### 4.6 MechanismInput

| Field | Type | Meaning |
| --- | --- | --- |
| `ready` | bool | Payload controller healthy |
| `fault` | bool | Forces safe return and commands mechanism `SAFE` |
| `first_aid_loaded` | bool | Required before start |
| `water_collection_complete` | bool | Collection action finished |
| `water_volume_l` | float | Measured collected volume; score caps at 2 L |
| `water_release_complete` | bool | Water release physically confirmed |
| `first_aid_armed` | bool | Release actuator armed and feedback healthy |
| `first_aid_release_complete` | bool | Kit no longer retained by aircraft |

## 5. Output ports

Every `step()` returns one `FSMOutput`. Consumers should treat it as the desired current command,
not as a pulse.

| Field | Consumer | Meaning |
| --- | --- | --- |
| `state`, `active_mission` | All teams/UI/logging | Current macro state and mission number |
| `flight_command` | Autopilot bridge | Hold, takeoff, follow route, descend, precision land, emergency land, disarmed |
| `route_id`, `route`, `route_repeat` | Autopilot bridge | Ordered WGS84 waypoints, acknowledgement key, and loop flag |
| `target_altitude_agl_m` | Autopilot bridge | Takeoff/descent setpoint where relevant |
| `vision_mode` | Vision team | Enables only the detector/localiser needed in this phase |
| `sound_mode` | Sound team | Enables/disables the dead-man alarm pipeline |
| `mechanism_command` | Mechanism team | Safe/prepare/collect/release/arm command |
| `export_results` | Mapping/reporting | Export map and GPS result tables immediately after landing |
| `completed_missions` | UI/reporting | Latched mission completion set |
| `accepted_vehicle_count`, `accepted_hotspot_count` | UI/reporting | Deduplicated scoring-valid records |
| `report_deadline_remaining_s` | UI/reporting | Countdown from 300 s after landing |
| `slot_deadline_remaining_s` | UI/logging | Countdown from the 30-minute competition slot start |
| `status` | All teams/UI/logging | Human-readable reason or current wait condition |

The autopilot/precision-landing module retains all low-level control authority. For multicopter/VTOL,
the FSM requests `PRECISION_LAND` only after ArUco ID 0 is visible; it does not command motors from
image pixels. For fixed-wing, the adapter performs the landing-area approach without the marker mode.

## 6. State-by-state contract

| State | Required input/transition guard | Main outputs |
| --- | --- | --- |
| `PRE_FLIGHT` | `start` plus Autopilot, Vision, Mapping, Sound, Mechanism ready; GPS OK; kit loaded | HOLD, mechanism SAFE |
| `TAKEOFF` | `airborne=true` | TAKEOFF to configured AGL altitude; arm kit |
| `M3_TRANSIT_WATER_SOURCE` | `completed_route_id=m3_water_source` | FOLLOW_ROUTE; water-source vision; prepare collector |
| `M3_COLLECT_WATER` | source aligned, then collection complete and volume `>0` | HOLD_POSITION; PREPARE until aligned, then COLLECT_WATER |
| `M3_TRANSIT_DROP_TARGET` | `completed_route_id=m3_water_target` | FOLLOW_ROUTE; water-target vision |
| `M3_ALIGN_WATER_TARGET` | `water_target_aligned=true` | HOLD_POSITION over red target |
| `M3_RELEASE_WATER` | physical release confirmation | RELEASE_WATER; latch Mission 3 complete |
| `M1_SURVEY_AREA_1` | route complete and Area 1 coverage complete | Area 1 lawnmower route; map/vehicle mode |
| `M1_SURVEY_AREA_2` | route complete and Area 2 coverage complete | Area 2 lawnmower route; map/vehicle mode; latch Mission 1 |
| `M2_SEARCH_HOTSPOTS` | two unique fixes with error `<=5 m` | Repeat fire-zone route; thermal localisation mode; latch Mission 2 |
| `M4_SEARCH_DEADMAN` | valid 2.6-3.0 kHz sound fix or LED-derived fix | Search route; sound + mannequin/LED vision; kit armed |
| `M4_APPROACH_DEADMAN` | dynamic approach route complete | FOLLOW_ROUTE to localised correct mannequin |
| `M4_DESCEND_FOR_DROP` | altitude `<=2 m`, horizontal error within configured goal, kit armed | DESCEND to 1.5 m default; drop-alignment vision |
| `M4_RELEASE_FIRST_AID` | physical release confirmation | RELEASE_FIRST_AID; latch Mission 4 |
| `RETURN_HOME` | home route complete | FOLLOW_ROUTE home; mechanism SAFE |
| `PRECISION_LANDING` | Autopilot `landed=true` | PRECISION_LAND; ArUco 5x5 ID 0 mode |
| `POST_FLIGHT_EXPORT` | map export-ready and export/submission complete | DISARMED; export trigger; five-minute countdown |
| `COMPLETE` | terminal | DISARMED |
| `EMERGENCY_LAND` | abort or geofence violation | EMERGENCY_LAND; all mechanisms SAFE |

Low battery, mechanism fault, `force_return`, or reaching the configured three-minute return margin
before the 30-minute slot ends moves any active task to `RETURN_HOME`. This may leave the current
mission incomplete. A geofence violation or explicit abort requests immediate emergency landing,
matching the rulebook safety priority.

## 7. Waypoint plan

Exact zone and target GPS coordinates are supplied on competition day, so they must not be guessed
in source code. Copy `config/mission_plan.template.json`, fill every route, then validate it:

```bash
python imav2026_outdoor_fsm.py --plan config/mission_plan.day_of.json --validate-plan
```

Required route IDs:

| Route ID | Minimum content |
| --- | --- |
| `m3_water_source` | Safe approach waypoint over/near the basin |
| `m3_water_target` | Safe approach over the red 1.5 x 1.5 m target, 25 m from basin |
| `m1_area1` | Camera-footprint lawnmower route for the 440 x 280 m area |
| `m1_area2` | Additional 440 x 320 m lawnmower route; enabled by default |
| `m2_fire_search` | Thermal-camera coverage route for the red fire zone |
| `m4_deadman_search` | Search pattern within 25 m of the supplied centre |
| `home` | Safe return waypoint above the day-specific landing zone |

`generate_lawnmower_route()` creates deterministic alternating endpoints for an approximately
rectangular area. Vaishnavi/Ilias/Kory must choose mapping altitude and lane spacing from calibrated
camera footprint and required image overlap. The M4 search spacing should be agreed with the sound
team from measured alarm detection range; the rulebook gives alarm power at 3 m but not a guaranteed
maximum detection range.

Before flight, plot every route, confirm it remains inside the black flight boundary/geofence, verify
terrain clearance, and upload the identical route IDs to the Autopilot bridge.

The default plan also contains the 1800 s slot duration, a conservative 180 s return margin, and a
10 m AGL M4 approach altitude before the gated descent. Adjust these only after measured flight-time,
wind, terrain and payload tests. Plan validation rejects takeoff/waypoint altitudes above the 80 m AGL
rulebook ceiling.

## 8. Integration skeleton

```python
plan = load_plan_json("config/mission_plan.day_of.json")
fsm = OutdoorMissionFSM(plan)

while middleware_is_running:
    inputs = collect_complete_fsm_input_snapshot()
    outputs = fsm.step(inputs)
    publish_to_autopilot(outputs.flight_command, outputs.route_id, outputs.route)
    publish_vision_mode(outputs.vision_mode)
    publish_sound_mode(outputs.sound_mode)
    publish_mechanism_command(outputs.mechanism_command)
    publish_status(outputs)
```

No module should infer mission state from time or waypoint number. Always use the published `state`
and return explicit readiness/completion feedback.

## 9. Rulebook requirements represented as gates

- Mission slot: 30 minutes - external UI should display a whole-flight countdown.
- Mission 1: map Area 1/2, identify vehicles, GPS error <=5 m, export within 5 minutes after landing.
- Mission 2: exactly one drone, two heat-point GPS fixes <=5 m, export within 5 minutes after landing.
- Mission 3: visible/transparent water system is a hardware requirement; score uses final volume up to 2 L.
- Mission 4: three mannequins within 25 m; correct one has 2.6-3.0 kHz/95 dB-at-3-m alarm and two LEDs;
  drop must be from <=2 m. The 0.5 m horizontal goal targets maximum distance score.
- Precision landing for multicopter: 1 x 1 m precision area, 3 x 3 m landing zone, 60 x 60 cm ArUco ID 0.
- Maximum outdoor altitude: 80 m AGL; maximum outdoor mass: 5 kg. These must also be enforced by
  flight-controller configuration, not only by this macro FSM.

## 10. Integration acceptance checklist

1. Each team can serialise/deserialise the exact dataclasses or equivalent middleware messages.
2. Route completion includes the route ID and cannot accidentally acknowledge the next route.
3. Vehicle/hotspot observations include stable IDs and conservative GPS error estimates.
4. Mission 4 release is hardware-inhibited above 2 m even if the FSM or communications fail.
5. Geofence and low-battery return/landing behaviours are independently enforced by Autopilot.
6. M1/M2 export is rehearsed and finishes in under five minutes after landing.
7. A hardware-in-the-loop test completes the full state sequence and all failure branches.

## 11. Explicit interpretation decisions

- Mission 1 prose says GPS coordinates must be supplied no later than five minutes after landing,
  while Table 7 also lists a lower submission score at 30 minutes. The FSM deliberately targets the
  stricter five-minute limit and exposes a 300 s countdown.
- Mission 1 gives incremental points for up to eight correctly identified/localised vehicles. The FSM
  records every valid unique vehicle but does not block all later missions if fewer than eight are found
  after the complete mapping route. A repeat/supplementary M1 pass is an operator planning decision.
- Mission 3 scores any delivered volume up to 2 L. The FSM accepts a positive confirmed collection so
  one weak collection does not block Missions 1/2/4; the configured goal remains 2 L.
- Mission 4 validation requires release at no more than 2 m. The configured 0.5 m horizontal goal is a
  scoring target, not a rulebook validity threshold.
- The marker landing flow applies to multicopter/VTOL. Fixed-wing uses its rulebook landing area and
  bypasses the ID 0 acquisition gate.
