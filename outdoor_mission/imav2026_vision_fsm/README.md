# IMAV 2026 Outdoor Mission Orchestrator

This package implements a typed macro FSM for completing all four outdoor missions in one flight:

1. collect and drop water;
2. map the field and record located vehicle identities;
3. locate two thermal hot spots;
4. localise the dead-man mannequin and drop the first-aid kit;
5. return, precision-land on ArUco ID 0, and export scoring data.

The implementation is deliberately decoupled from OpenCV, detectors, sound localisation, payload
drivers and low-level flight control. See `TEAM_INTERFACE.md` for every state and input/output port.

## Files

- `imav2026_outdoor_fsm.py` - typed input/output ports and macro state machine.
- `simple_waypoint_planner.py` - validated route table and deterministic lawnmower helper.
- `config/mission_plan.template.json` - copy and fill with coordinates supplied on competition day.
- `TEAM_INTERFACE.md` - integration contract for all module teams.
- `tests/test_fsm.py` - full four-mission flow and guard/safety tests.

## Validate a day-of-competition plan

```bash
python imav2026_outdoor_fsm.py \
  --plan config/mission_plan.day_of.json \
  --validate-plan
```

The committed template intentionally contains empty routes and therefore fails validation until the
day-specific coordinates are filled. This prevents placeholder coordinates from reaching an aircraft.

## Test

```bash
python -m unittest discover -s tests -v
```
