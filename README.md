# ulog-why

**Find the earliest likely cause in a PX4 flight log, then the chain up to the failsafe or crash.**

When a PX4 vehicle fails, the loudest event in the log (failsafe, tilt limit, "landing detected") is usually the end of the story. `ulog-why` reads a `.ulg`, builds a timeline of anomalies across sensors, estimator, power, propulsion, datalink, control and failsafe state, and ranks which one came first and is best supported by evidence. It came out of [this PX4 forum thread](https://discuss.px4.io/t/ulog-crash-analysis-how-do-you-identify-the-first-causal-failure-instead-of-the-final-failsafe/49534).

One Rust binary, offline, no Python. The ULog parser is written from the [PX4 ULog spec](https://docs.px4.io/main/en/dev_log/ulog_file_format.html) (nested types, omitted trailing padding, appended data, truncated tails, refuses unknown incompat bits).

```sh
curl -fsSL https://raw.githubusercontent.com/reacherwu/ulog-why/main/install.sh | sh   # needs cargo
ulog-why demo                  # bundled 857 KB SITL log, works offline
ulog-why flight.ulg            # human report
ulog-why flight.ulg --json     # machine-readable (verdict + full timeline)
ulog-why flight.ulg --timeline # also print every detected event
ulog-why trim in.ulg out.ulg 10  # shrink a log to the topics ulog-why reads, 10 Hz
```

## Example (`ulog-why demo`)

```
EARLIEST LIKELY CAUSE: GPS  (confidence 100%)
  T +20.54s  GPS fix lost (fix_type 3 -> 0)   [vehicle_gps_position]
  first failsafe / loss of control: T +25.99s  failsafe activated  (5.5 s after the cause)

Chain (cause -> outcome):
   1. T +20.54s  gps        cause?   GPS fix lost (fix_type 3 -> 0)
   2. T +20.54s  gps        cause?   GPS satellites dropped 10 -> 0
   4. T +21.46s  estimator  symptom  EKF dead-reckoning on inertial only
   6. T +25.46s  failsafe   response log: [mc_pos_control] Failsafe: blind land
   ...
  11. T +25.99s  failsafe   response mode AUTO_LOITER -> DESCEND
```

"Confidence" is this category's share of the evidence score among candidates. It is not a calibrated probability.

## What it looks at

| Area | Signals |
|---|---|
| GPS | fix_type drop, satellites, eph, data gaps, EKF GPS innovation ratios, `cs_gps` fusion stop |
| Baro / mag / IMU | data gaps, implausible baro altitude steps, field-strength steps, innovation ratios, EKF fault/reject flags, accel clipping |
| Battery | voltage sag, `warning` level, failsafe battery flags |
| Propulsion | sustained motor saturation (`actuator_motors`), failure detector motor/prop flags |
| Control | attitude vs setpoint error > 20°, altitude below setpoint, tilt > 60°, impact accel |
| Link | GCS heartbeat stop (`telemetry_status`), `gcs_connection_lost`, RC loss |
| State | failsafe, mode changes, disarm, warning/error log messages, in-flight parameter changes, logger dropouts |

Ranking: the anchor is the first failsafe or loss of control. Among cause-capable categories seen before it, earlier onsets win, weighted by how direct the evidence is (raw sensor > estimator flag > log text). Innovation failures already present at arming are reported as a standing condition, not a fault onset (a point raised in the forum thread).

## Validation (PX4 v1.15.4 SITL, SIH quadrotor)

`sim/gen_dataset.py` flies a fresh SITL instance per trial (take off to 15–25 m, reposition 40–120 m), injects one known fault at a random time, and records the ground-truth label and injection time. Faults: GPS loss (`SIM_GPS_USED=0`), baro offset (±300–2000 Pa), mag offset (±0.4–1.5 G), thrust loss (`SIH_T_MAX` 1.6–2.7 N per motor), battery failure (`MAV_CMD_INJECT_FAILURE`), GCS datalink loss (heartbeats stopped), and no fault.

A trial counts as correct when the category matches and the reported onset is within −1 s/+15 s of the injection (for "no fault": nothing reported).

- **Dev set** (rules tuned here): 14/14 at freeze time; 16/17 including 3 dev trials that finished after the freeze.
- **Held-out set** (4 seeds, generated after the rules were frozen at `cd9546f`, scored once): **45/50 (90%)**; category right in 47/50. Median detection delay 0.1 s, max 2.4 s.

| Fault | Held-out |
|---|---|
| GPS loss | 7/7 |
| No fault | 7/7 (0 false alarms) |
| Thrust loss | 6/6 |
| Mag offset | 7/8 |
| Datalink loss | 7/8 |
| Baro offset | 6/7 |
| Battery failure | 5/7 |

The five misses: a 0.48 G mag offset that only raised innovation ratios and never caused a failsafe (below the report threshold); two takeoff transients (an attitude-error/saturation burst and a 42 m/s² accel spike right after liftoff) that were taken as the anchor and blamed before the real fault; two battery cases with the right category whose onset was put 1.0–1.3 s before the injection. Six further held-out trials failed to arm in SITL and were dropped before scoring (no fault had been injected). Raw labels and per-trial results are in `results/`.

**Parser check:** per-subscription sample counts match [pyulog](https://github.com/PX4/pyulog) exactly on its 6 public test logs (PX4 v1.6–v1.15, including appended-data logs) and on SITL logs (`results/pyulog_crosscheck.txt`). On those public logs, which have no ground truth, `ulog-why` reports no cause for all 6; one battery false positive on `sample_px4_events.ulg` was found during development and fixed.

## Honest limits

- Every number above comes from simulated, abrupt, single faults on one airframe in SIH. Real crashes have vibration, gradual failures, multiple faults and missing topics. Expect lower accuracy.
- The rules are hand-written thresholds. Time order is evidence, not proof of causality.
- Takeoff transients can be mistaken for the start of an incident (2 of the 5 held-out misses).
- Real motor/ESC failures were not tested: SIH does not apply `failure motor off` physically, so thrust loss was simulated with `SIH_T_MAX`.
- Simulator-only signals are excluded so they cannot leak the answer: `SIM_*`/`SIH_*` parameter changes, simulator and failure-injection log lines, ground-truth topics.
- Tested on PX4 v1.15 multicopter logs. Fixed-wing/VTOL-specific signals are not covered. Older logs parse but lack many topics.

## Prior art

- [PX4 Flight Review](https://docs.px4.io/main/en/log/flight_review): the standard web plots for ULog. `ulog-why` tells you which plots to open first.
- [flightdiff](https://github.com/Shuaibu78/flightdiff): Rust offline forensics, focused on diffing parameters/logs between flights.
- [FlightMD](https://github.com/Praddyx15/FlightMD): rule-based health reports for PX4/ArduPilot logs (Python).

## Reproduce

```sh
cargo build --release && cargo test --release
# needs PX4-Autopilot v1.15.4 built with `make px4_sitl_default` and pymavlink
python sim/gen_dataset.py --px4 ~/PX4-Autopilot --out data/held --seed 11 --per-fault 2
python sim/evaluate.py data/held -v
```

SITL runs in real time, ~2 min per trial. MIT license.
