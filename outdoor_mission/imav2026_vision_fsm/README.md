# IMAV 2026 Outdoor M1/M4 Pluggable FSM

This package follows `Rulebook_IMAV2026_V4-1.pdf` and supports three profiles with one codebase:

- Mission 1 test: mapping and vehicle identification only;
- Mission 4 test: mannequin search and first-aid drop only;
- Combined competition run: Mission 1, then Mission 4, then one safe return/landing.

Mission 2 and Mission 3 are not part of the current team objective.

## Structure

- `imav2026_outdoor_fsm.py` - mission plugins, typed ports and macro orchestrator.
- `simple_waypoint_planner.py` - profile-aware plan validation and lawnmower routes.
- `config/mission_plan.template.json` - day-of-competition route template.
- `TEAM_INTERFACE.md` - state and module integration contract.
- `tests/test_fsm.py` - M1-only, M4-only, combined and safety tests.

## Select a profile

The JSON defaults to `[1, 4]`. The CLI can override it without changing code:

```bash
# Mission 1 test
python imav2026_outdoor_fsm.py --plan config/mission_plan.day_of.json --missions 1 --validate-plan

# Mission 4 test
python imav2026_outdoor_fsm.py --plan config/mission_plan.day_of.json --missions 4 --validate-plan

# Combined final FSM
python imav2026_outdoor_fsm.py --plan config/mission_plan.day_of.json --missions 1 4 --validate-plan
```

Plan validation is profile-aware: M1-only does not require the M4 search route, and M4-only does not
require either mapping route. Every profile requires `home`.

The committed template deliberately has empty routes because exact zone and landing coordinates are
provided on the competition day. Copy it to `mission_plan.day_of.json`, fill only real coordinates,
plot/check the route against the geofence, and validate it before connecting an aircraft.

## Test

```bash
python -m unittest discover -s tests -v
```
