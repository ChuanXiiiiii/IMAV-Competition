# IMAV 2026 Outdoor Mission FSM

This simulator follows sections 5.2-5.4 of `Rulebook_IMAV2026_V2.pdf`. Each outdoor mission is a
separate flight, selected with `--mission`. Return and precision landing use the rulebook's 5x5 ArUco
ID 0 landing marker. Mission 1 and 2 continue to a five-minute post-landing report state.

## Run

```bash
pip install -r requirements.txt
python imav2026_outdoor_fsm.py --mission 1 --camera 0
```

Mission 4 accepts a simulated release altitude:

```bash
python imav2026_outdoor_fsm.py --mission 4 --drop-altitude 1.5
```

## Simulation controls

| Key | Event |
| --- | --- |
| `t` | Take off / start selected mission |
| `x` | Safety abort |
| `q` | Quit simulator |
| `1`, `2` | Mission 1: Mapping Area 1 / Area 2 complete |
| `v` | Mission 1: identified vehicle and GPS position recorded (up to 8) |
| `f` | Mission 1: mapping finished, return |
| `h` | Mission 2: hot spot and GPS position recorded (twice) |
| `c`, `d` | Mission 3: water collected / water dropped into target |
| `b` | Mission 4: dead-man alarm or LEDs detected |
| `p` | Mission 4: first-aid package released at `--drop-altitude` |
| `s` | Mission 1/2: post-landing report submitted |

During return, showing ArUco ID 0 changes the FSM to precision landing. The marker must remain
within 30 pixels of the camera-frame centre for five seconds.
