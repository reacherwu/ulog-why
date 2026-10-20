//! Anomaly detectors + earliest-cause ranking.
use crate::ulog::{Topic, ULog};
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, PartialOrd, Ord, Hash)]
#[serde(rename_all = "snake_case")]
pub enum Cat { Gps, Baro, Mag, Imu, Battery, Propulsion, Datalink, Rc, Config, Estimator, Control, Failsafe, Crash, Logging }
impl Cat {
    pub fn name(self) -> &'static str {
        match self {
            Cat::Gps => "gps", Cat::Baro => "baro", Cat::Mag => "mag", Cat::Imu => "imu", Cat::Battery => "battery",
            Cat::Propulsion => "propulsion", Cat::Datalink => "datalink", Cat::Rc => "rc", Cat::Config => "config",
            Cat::Estimator => "estimator", Cat::Control => "control", Cat::Failsafe => "failsafe", Cat::Crash => "crash",
            Cat::Logging => "logging",
        }
    }
    /// categories that can be a root cause
    pub fn causal(self) -> bool {
        matches!(self, Cat::Gps | Cat::Baro | Cat::Mag | Cat::Imu | Cat::Battery | Cat::Propulsion | Cat::Datalink | Cat::Rc | Cat::Config)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum Role { Cause, Symptom, Response, Terminal }

#[derive(Clone, Debug, Serialize)]
pub struct Event {
    /// seconds since boot
    pub t: f64,
    pub cat: Cat,
    pub role: Role,
    /// 0..1: how direct the evidence is (raw sensor > estimator > log text)
    pub weight: f64,
    pub what: String,
    pub source: String,
}

fn ev(t: f64, cat: Cat, role: Role, weight: f64, what: impl Into<String>, source: &str) -> Event {
    Event { t, cat, role, weight, what: what.into(), source: source.to_string() }
}

fn secs(t: &Topic) -> Vec<f64> { t.ts().into_iter().map(|x| x as f64 * 1e-6).collect() }

/// onset times of runs where pred holds for at least `dur` seconds (reported at run start)
fn sustained(ts: &[f64], pred: &[bool], dur: f64) -> Vec<(f64, f64)> {
    let mut out = vec![];
    let mut start: Option<usize> = None;
    for i in 0..=ts.len() {
        let on = i < ts.len() && pred[i];
        match (on, start) {
            (true, None) => start = Some(i),
            (false, Some(s)) => {
                let end = ts[i - 1];
                if end - ts[s] >= dur { out.push((ts[s], end)); }
                start = None;
            }
            _ => {}
        }
    }
    out
}

/// rising edges of a 0/1 column
fn rises(ts: &[f64], v: &[f64]) -> Vec<f64> {
    (1..v.len()).filter(|&i| v[i] > 0.5 && v[i - 1] <= 0.5).map(|i| ts[i]).collect()
}
fn falls(ts: &[f64], v: &[f64]) -> Vec<f64> {
    (1..v.len()).filter(|&i| v[i] <= 0.5 && v[i - 1] > 0.5).map(|i| ts[i]).collect()
}

/// gaps much longer than the topic's median period
fn gaps(ts: &[f64], min_gap: f64) -> Vec<(f64, f64)> {
    if ts.len() < 10 { return vec![]; }
    let mut d: Vec<f64> = ts.windows(2).map(|w| w[1] - w[0]).collect();
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = d[d.len() / 2];
    let thr = (med * 10.0).max(min_gap);
    ts.windows(2).filter(|w| w[1] - w[0] > thr).map(|w| (w[0], w[1] - w[0])).collect()
}

pub struct Flight { pub arm: Option<f64>, pub disarm: Option<f64>, pub airborne: Vec<(f64, f64)>, pub end: f64 }

impl Flight {
    fn in_air(&self, t: f64) -> bool { self.airborne.iter().any(|&(a, b)| t >= a + 1.0 && t <= b) }
    fn armed(&self, t: f64) -> bool {
        match self.arm { Some(a) => t >= a && self.disarm.is_none_or(|d| t <= d + 0.5), None => false }
    }
}

pub fn flight(u: &ULog) -> Flight {
    let end = u.last_ts as f64 * 1e-6;
    let mut arm = None; let mut disarm = None;
    if let Some(t) = u.topic("vehicle_status") {
        let ts = secs(t);
        if let Some(st) = t.col("arming_state") {
            for i in 0..st.len() {
                if st[i] == 2.0 && arm.is_none() { arm = Some(ts[i]); }
                if st[i] != 2.0 && arm.is_some() && disarm.is_none() && ts[i] > arm.unwrap() { disarm = Some(ts[i]); }
            }
        }
    }
    let mut airborne = vec![];
    if let Some(t) = u.topic("vehicle_land_detected") {
        let ts = secs(t);
        if let Some(l) = t.col("landed") {
            let mut s: Option<f64> = None;
            for i in 0..l.len() {
                if l[i] < 0.5 && s.is_none() { s = Some(ts[i]); }
                if l[i] > 0.5 { if let Some(a) = s.take() { airborne.push((a, ts[i])); } }
            }
            if let Some(a) = s { airborne.push((a, end)); }
        }
    }
    Flight { arm, disarm, airborne, end }
}

const NAV: &[(u32, &str)] = &[(0, "MANUAL"), (1, "ALTCTL"), (2, "POSCTL"), (3, "AUTO_MISSION"), (4, "AUTO_LOITER"),
    (5, "AUTO_RTL"), (10, "ACRO"), (12, "DESCEND"), (13, "TERMINATION"), (14, "OFFBOARD"), (15, "STAB"),
    (17, "AUTO_TAKEOFF"), (18, "AUTO_LAND"), (19, "AUTO_FOLLOW_TARGET"), (20, "AUTO_PRECLAND"), (21, "ORBIT")];
fn nav_name(n: f64) -> String {
    NAV.iter().find(|x| x.0 as f64 == n).map(|x| x.1.to_string()).unwrap_or(format!("nav_state {n}"))
}

/// Messages printed by simulator modules or the failure-injection command are never evidence.
fn sim_only_msg(text: &str) -> bool {
    text.contains("INJECT_FAILURE") || text.starts_with("[battery_simulator]") || text.starts_with("[simulator")
        || text.starts_with("[sensor_") && text.contains("_sim]") || text.starts_with("[failure]")
}

/// Simulator-only parameters are never used as evidence (they would leak the injected fault).
fn sim_only(p: &str) -> bool { p.starts_with("SIM_") || p.starts_with("SIH_") }

pub fn detect(u: &ULog, f: &Flight) -> Vec<Event> {
    let mut e: Vec<Event> = vec![];
    let in_win = |t: f64| f.armed(t);

    // ---------- GPS ----------
    if let Some(t) = u.topic("vehicle_gps_position").or(u.topic("sensor_gps")) {
        let ts = secs(t);
        if let Some(fix) = t.col("fix_type") {
            for i in 1..fix.len() {
                if fix[i] < 3.0 && fix[i - 1] >= 3.0 && in_win(ts[i]) {
                    e.push(ev(ts[i], Cat::Gps, Role::Cause, 1.0, format!("GPS fix lost (fix_type {} -> {})", fix[i - 1], fix[i]), &t.name));
                }
            }
        }
        if let Some(sat) = t.col("satellites_used") {
            for i in 1..sat.len() {
                if sat[i] < 6.0 && sat[i - 1] >= 6.0 && in_win(ts[i]) {
                    e.push(ev(ts[i], Cat::Gps, Role::Cause, 0.9, format!("GPS satellites dropped {} -> {}", sat[i - 1], sat[i]), &t.name));
                }
            }
        }
        if let Some(eph) = t.col("eph") {
            let bad: Vec<bool> = eph.iter().map(|&x| x > 5.0).collect();
            for (a, _) in sustained(&ts, &bad, 1.0) { if in_win(a) {
                e.push(ev(a, Cat::Gps, Role::Cause, 0.7, "GPS horizontal accuracy (eph) > 5 m", &t.name));
            } }
        }
        for (a, g) in gaps(&ts, 1.0) { if in_win(a) {
            e.push(ev(a, Cat::Gps, Role::Cause, 0.9, format!("GPS data stopped for {g:.1} s"), &t.name));
        } }
    }
    // ---------- baro ----------
    if let Some(t) = u.topic("vehicle_air_data") {
        let ts = secs(t);
        for (a, g) in gaps(&ts, 1.0) { if in_win(a) {
            e.push(ev(a, Cat::Baro, Role::Cause, 0.9, format!("baro data stopped for {g:.1} s"), &t.name));
        } }
        if let Some(alt) = t.col("baro_alt_meter") {
            // implausible step: > 8 m change within 0.3 s
            let mut j = 0;
            let mut last_ev = -10.0;
            for i in 0..alt.len() {
                while ts[i] - ts[j] > 0.3 { j += 1; }
                let d = alt[i] - alt[j];
                if d.abs() > 8.0 && in_win(ts[i]) && ts[i] - last_ev > 5.0 {
                    e.push(ev(ts[j], Cat::Baro, Role::Cause, 1.0, format!("baro altitude jumped {d:+.1} m in {:.2} s", ts[i] - ts[j]), &t.name));
                    last_ev = ts[i];
                }
            }
        }
    }
    // ---------- mag ----------
    if let Some(t) = u.topic("vehicle_magnetometer") {
        let ts = secs(t);
        for (a, g) in gaps(&ts, 1.0) { if in_win(a) {
            e.push(ev(a, Cat::Mag, Role::Cause, 0.9, format!("magnetometer data stopped for {g:.1} s"), &t.name));
        } }
        if let (Some(x), Some(y), Some(z)) = (t.col("magnetometer_ga[0]"), t.col("magnetometer_ga[1]"), t.col("magnetometer_ga[2]")) {
            let n: Vec<f64> = (0..x.len()).map(|i| (x[i] * x[i] + y[i] * y[i] + z[i] * z[i]).sqrt()).collect();
            // step in field strength vs the previous 3 s median
            let mut last_ev = -10.0;
            let mut j = 0;
            for i in 0..n.len() {
                while ts[i] - ts[j] > 3.0 { j += 1; }
                if i - j < 5 { continue; }
                let mut w: Vec<f64> = n[j..i].to_vec();
                w.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let med = w[w.len() / 2];
                if (n[i] - med).abs() > 0.15 && (n[i] - med).abs() / med > 0.25 && in_win(ts[i]) && ts[i] - last_ev > 5.0 {
                    e.push(ev(ts[i], Cat::Mag, Role::Cause, 1.0, format!("magnetic field strength jumped {med:.2} -> {:.2} G", n[i]), &t.name));
                    last_ev = ts[i];
                }
            }
        }
    }
    // ---------- IMU ----------
    if let Some(t) = u.topic("sensor_combined") {
        let ts = secs(t);
        for (a, g) in gaps(&ts, 0.2) { if in_win(a) {
            e.push(ev(a, Cat::Imu, Role::Cause, 0.9, format!("IMU data stopped for {g:.2} s"), &t.name));
        } }
        if let (Some(x), Some(y), Some(z)) = (t.col("accelerometer_m_s2[0]"), t.col("accelerometer_m_s2[1]"), t.col("accelerometer_m_s2[2]")) {
            let mut last = -10.0;
            for i in 0..x.len() {
                let n = (x[i] * x[i] + y[i] * y[i] + z[i] * z[i]).sqrt();
                if n > 30.0 && in_win(ts[i]) && ts[i] - last > 2.0 {
                    e.push(ev(ts[i], Cat::Crash, Role::Terminal, 1.0, format!("impact: accel {n:.0} m/s^2"), &t.name));
                    last = ts[i];
                }
            }
        }
    }
    // ---------- estimator innovations ----------
    if let Some(t) = u.topic("estimator_innovation_test_ratios") {
        let ts = secs(t);
        let groups: &[(Cat, &str, &[&str])] = &[
            (Cat::Gps, "GPS position/velocity", &["gps_hpos[0]", "gps_hpos[1]", "gps_vpos", "gps_hvel[0]", "gps_hvel[1]", "gps_vvel"]),
            (Cat::Baro, "baro height", &["baro_vpos"]),
            (Cat::Mag, "magnetometer/heading", &["mag_field[0]", "mag_field[1]", "mag_field[2]", "heading"]),
        ];
        for (cat, label, fields) in groups {
            let cols: Vec<Vec<f64>> = fields.iter().filter_map(|n| t.col(n)).collect();
            if cols.is_empty() { continue; }
            let pred: Vec<bool> = (0..ts.len()).map(|i| cols.iter().any(|c| c[i] > 1.0)).collect();
            for (a, b) in sustained(&ts, &pred, 0.5) {
                if in_win(a) && f.in_air(a) {
                    e.push(ev(a, *cat, Role::Symptom, 0.6, format!("EKF {label} innovation test failing (ratio > 1 for {:.1} s)", b - a), &t.name));
                } else if let Some(arm) = f.arm {
                    // already failing before/at arming: a standing condition, not a fault onset
                    if a < arm + 1.0 && b > arm + 1.0 {
                        e.push(ev(arm, Cat::Logging, Role::Symptom, 0.2,
                            format!("standing condition: EKF {label} innovation test already failing at arming"), &t.name));
                    }
                }
            }
        }
    }
    if let Some(t) = u.topic("estimator_status_flags") {
        let ts = secs(t);
        let fl: &[(&str, bool, Cat, &str)] = &[
            ("cs_gps", false, Cat::Gps, "EKF stopped fusing GPS"),
            ("cs_baro_hgt", false, Cat::Baro, "EKF stopped fusing baro height"),
            ("cs_mag_fault", true, Cat::Mag, "EKF declared magnetometer fault"),
            ("cs_mag_field_disturbed", true, Cat::Mag, "EKF: magnetic field disturbed"),
            ("cs_inertial_dead_reckoning", true, Cat::Estimator, "EKF dead-reckoning on inertial only"),
            ("reject_hor_pos", true, Cat::Gps, "EKF rejecting horizontal position"),
            ("reject_ver_pos", true, Cat::Baro, "EKF rejecting vertical position"),
            ("reject_yaw", true, Cat::Mag, "EKF rejecting yaw"),
            ("fs_bad_acc_vertical", true, Cat::Imu, "EKF: bad vertical accel data"),
            ("fs_bad_acc_clipping", true, Cat::Imu, "EKF: accelerometer clipping"),
        ];
        for (name, rising, cat, what) in fl {
            if let Some(c) = t.col(name) {
                let edges = if *rising { rises(&ts, &c) } else { falls(&ts, &c) };
                for a in edges { if in_win(a) && f.in_air(a) { e.push(ev(a, *cat, Role::Symptom, 0.5, *what, &t.name)); } }
            }
        }
    }
    // ---------- battery ----------
    if let Some(t) = u.topic("battery_status") {
        let ts = secs(t);
        if let Some(w) = t.col("warning") {
            for i in 1..w.len() {
                if w[i] > w[i - 1] && in_win(ts[i]) {
                    let lvl = ["none", "low", "critical", "emergency", "failed"].get(w[i] as usize).copied().unwrap_or("?");
                    e.push(ev(ts[i], Cat::Battery, Role::Cause, 0.8, format!("battery warning -> {lvl}"), &t.name));
                }
            }
        }
        if let (Some(v), Some(r)) = (t.col("voltage_v"), t.col("remaining")) {
            let mut j = 0;
            let mut last = -10.0;
            for i in 0..v.len() {
                while ts[i] - ts[j] > 3.0 { j += 1; }
                let dv = v[j] - v[i]; let dr = r[j] - r[i];
                if (dv > 0.08 * v[j].max(0.1) || (dr > 0.15 && dv > 0.03 * v[j].max(0.1))) && in_win(ts[i]) && ts[i] - last > 10.0 {
                    // onset: first sample in the window that already dropped noticeably
                    let k = (j..=i).find(|&k| v[j] - v[k] > 0.02 * v[j].max(0.1) || r[j] - r[k] > 0.03).unwrap_or(j);
                    e.push(ev(ts[k], Cat::Battery, Role::Cause, 1.0,
                        format!("battery sag: {:.2} V -> {:.2} V, remaining {:.0}% -> {:.0}% in {:.1} s", v[j], v[i], r[j] * 100.0, r[i] * 100.0, ts[i] - ts[j]), &t.name));
                    last = ts[i];
                }
            }
        }
    }
    // ---------- propulsion ----------
    if let Some(t) = u.topic("actuator_motors") {
        let ts = secs(t);
        let cols: Vec<Vec<f64>> = (0..12).filter_map(|k| t.col(&format!("control[{k}]"))).collect();
        let active: Vec<&Vec<f64>> = cols.iter().filter(|c| c.iter().any(|x| x.is_finite() && *x > 0.05)).collect();
        if !active.is_empty() {
            let any: Vec<bool> = (0..ts.len()).map(|i| active.iter().any(|c| c[i] > 0.97)).collect();
            let all: Vec<bool> = (0..ts.len()).map(|i| active.iter().all(|c| c[i] > 0.95)).collect();
            for (a, b) in sustained(&ts, &all, 0.5) { if in_win(a) && f.in_air(a) {
                e.push(ev(a, Cat::Propulsion, Role::Cause, 0.9, format!("all motors saturated high for {:.1} s (thrust limit)", b - a), &t.name));
            } }
            for (a, b) in sustained(&ts, &any, 1.0) { if in_win(a) && f.in_air(a) {
                let w = if b - a >= 5.0 { 0.9 } else { 0.7 };
                e.push(ev(a, Cat::Propulsion, Role::Cause, w, format!("motor output saturated for {:.1} s", b - a), &t.name));
            } }
        }
    }
    if let Some(t) = u.topic("failure_detector_status") {
        let ts = secs(t);
        for (n, cat, role, what) in [("fd_motor", Cat::Propulsion, Role::Cause, "failure detector: motor failure"),
                                      ("fd_roll", Cat::Control, Role::Symptom, "failure detector: roll limit exceeded"),
                                      ("fd_pitch", Cat::Control, Role::Symptom, "failure detector: pitch limit exceeded"),
                                      ("fd_battery", Cat::Battery, Role::Cause, "failure detector: battery failure"),
                                      ("fd_imbalanced_prop", Cat::Propulsion, Role::Cause, "failure detector: imbalanced prop")] {
            if let Some(c) = t.col(n) { for a in rises(&ts, &c) { if in_win(a) { e.push(ev(a, cat, role, 0.8, what, &t.name)); } } }
        }
    }
    // ---------- control tracking ----------
    if let (Some(lp), Some(sp)) = (u.topic("vehicle_local_position"), u.topic("vehicle_local_position_setpoint")) {
        let (lt, st) = (secs(lp), secs(sp));
        if let (Some(z), Some(zs)) = (lp.col("z"), sp.col("z")) {
            let mut k = 0;
            let pred: Vec<bool> = (0..lt.len()).map(|i| {
                while k + 1 < st.len() && st[k + 1] <= lt[i] { k += 1; }
                !st.is_empty() && zs[k].is_finite() && (z[i] - zs[k]) > 3.0 && (lt[i] - st[k]).abs() < 1.0
            }).collect();
            for (a, b) in sustained(&lt, &pred, 1.0) { if in_win(a) && f.in_air(a) {
                e.push(ev(a, Cat::Control, Role::Symptom, 0.5, format!("altitude below setpoint by > 3 m for {:.1} s", b - a), &lp.name));
            } }
        }
    }
    if let (Some(at), Some(sp)) = (u.topic("vehicle_attitude"), u.topic("vehicle_attitude_setpoint")) {
        let (a_t, s_t) = (secs(at), secs(sp));
        let q: Vec<Vec<f64>> = (0..4).filter_map(|k| at.col(&format!("q[{k}]"))).collect();
        let qd: Vec<Vec<f64>> = (0..4).filter_map(|k| sp.col(&format!("q_d[{k}]"))).collect();
        if q.len() == 4 && qd.len() == 4 && !s_t.is_empty() {
            let mut k = 0;
            let mut err = vec![false; a_t.len()];
            let mut tilt = vec![false; a_t.len()];
            for i in 0..a_t.len() {
                while k + 1 < s_t.len() && s_t[k + 1] <= a_t[i] { k += 1; }
                let dot: f64 = (0..4).map(|j| q[j][i] * qd[j][k]).sum::<f64>().abs().min(1.0);
                err[i] = 2.0 * dot.acos() > 20f64.to_radians() && (a_t[i] - s_t[k]).abs() < 0.5;
                // tilt from body z vs vertical
                let (w, x, y) = (q[0][i], q[1][i], q[2][i]);
                let cz = 1.0 - 2.0 * (x * x + y * y);
                let _ = w;
                tilt[i] = cz < 60f64.to_radians().cos();
            }
            for (a, b) in sustained(&a_t, &err, 0.3) { if in_win(a) && f.in_air(a) {
                e.push(ev(a, Cat::Control, Role::Symptom, 0.5, format!("attitude tracking error > 20 deg for {:.1} s", b - a), &at.name));
            } }
            for (a, _) in sustained(&a_t, &tilt, 0.2) { if in_win(a) {
                e.push(ev(a, Cat::Crash, Role::Terminal, 0.8, "vehicle tilted > 60 deg (loss of control)", &at.name));
            } }
        }
    }
    // ---------- link / failsafe / arming ----------
    for t in u.topics_named("telemetry_status") {
        let ts = secs(t);
        if let Some(c) = t.col("heartbeat_type_gcs") {
            for a in falls(&ts, &c) { if in_win(a) {
                e.push(ev(a, Cat::Datalink, Role::Cause, 0.9, format!("GCS heartbeats stopped on MAVLink instance {}", t.multi_id), &t.name));
            } }
        }
    }
    if let Some(t) = u.topic("vehicle_status") {
        let ts = secs(t);
        if let Some(c) = t.col("gcs_connection_lost") { for a in rises(&ts, &c) { if in_win(a) {
            e.push(ev(a, Cat::Datalink, Role::Cause, 1.0, "GCS datalink lost", &t.name));
        } } }
        if let Some(c) = t.col("failsafe") { for a in rises(&ts, &c) { if in_win(a) {
            e.push(ev(a, Cat::Failsafe, Role::Response, 1.0, "failsafe activated", &t.name));
        } } }
        if let Some(n) = t.col("nav_state") {
            for i in 1..n.len() { if n[i] != n[i - 1] && in_win(ts[i]) {
                e.push(ev(ts[i], Cat::Failsafe, Role::Response, 0.3, format!("mode {} -> {}", nav_name(n[i - 1]), nav_name(n[i])), &t.name));
            } }
        }
    }
    if let Some(t) = u.topic("failsafe_flags") {
        let ts = secs(t);
        for (n, cat, what) in [("manual_control_signal_lost", Cat::Rc, "RC signal lost"),
                               ("gcs_connection_lost", Cat::Datalink, "failsafe flag: GCS connection lost"),
                               ("battery_warning", Cat::Battery, "failsafe flag: battery warning"),
                               ("battery_unhealthy", Cat::Battery, "failsafe flag: battery unhealthy"),
                               ("global_position_invalid", Cat::Estimator, "global position invalid"),
                               ("local_position_invalid", Cat::Estimator, "local position invalid"),
                               ("local_altitude_invalid", Cat::Estimator, "local altitude invalid"),
                               ("attitude_invalid", Cat::Estimator, "attitude invalid"),
                               ("fd_critical_failure", Cat::Control, "critical failure detected"),
                               ("fd_motor_failure", Cat::Propulsion, "motor failure flagged")] {
            if let Some(c) = t.col(n) {
                for a in rises(&ts, &c) { if in_win(a) {
                    let role = if cat.causal() { Role::Cause } else { Role::Symptom };
                    e.push(ev(a, cat, role, 0.6, what, &t.name));
                } }
            }
        }
    }
    if let Some(d) = f.disarm { e.push(ev(d, Cat::Failsafe, Role::Terminal, 0.2, "disarmed", "vehicle_status")); }
    // ---------- log messages ----------
    for m in &u.messages {
        let t = m.ts as f64 * 1e-6;
        if m.level > 4 || !in_win(t) || sim_only_msg(&m.text) { continue; }
        let l = m.text.to_lowercase();
        let cat = if l.contains("gps") || l.contains("gnss") || l.contains("position") { Cat::Gps }
            else if l.contains("baro") || l.contains("height") || l.contains("altitude") { Cat::Baro }
            else if l.contains("mag") || l.contains("heading") || l.contains("yaw") || l.contains("compass") { Cat::Mag }
            else if l.contains("battery") || l.contains("voltage") || l.contains("power") { Cat::Battery }
            else if l.contains("motor") || l.contains("esc") || l.contains("thrust") { Cat::Propulsion }
            else if l.contains("data link") || l.contains("datalink") || l.contains("gcs") || l.contains("connection") { Cat::Datalink }
            else if l.contains("rc ") || l.contains("manual control") { Cat::Rc }
            else if l.contains("failsafe") { Cat::Failsafe }
            else { Cat::Logging };
        let role = if cat.causal() { Role::Cause } else { Role::Response };
        e.push(ev(t, cat, role, 0.4, format!("log: {}", m.text), "logged message"));
    }
    for &(ts, ms) in &u.dropouts {
        let t = ts as f64 * 1e-6;
        if in_win(t) && ms >= 200 { e.push(ev(t, Cat::Logging, Role::Symptom, 0.2, format!("logger dropout {ms} ms"), "ulog")); }
    }
    // ---------- in-flight parameter changes ----------
    for (p, ch) in &u.param_changes {
        if sim_only(p) { continue; }
        for c in ch {
            let t = c.ts as f64 * 1e-6;
            if in_win(t) && f.in_air(t) { e.push(ev(t, Cat::Config, Role::Cause, 0.5, format!("parameter {p} changed in flight -> {}", c.value), "parameter")); }
        }
    }
    e.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap());
    e
}

#[derive(Serialize, Debug)]
pub struct Verdict {
    pub root: Option<Event>,
    pub root_category: String,
    pub confidence: f64,
    pub anchor: Option<Event>,
    pub supporting: Vec<Event>,
    pub chain: Vec<Event>,
    pub alternatives: Vec<(String, f64)>,
}

/// Pick the earliest likely cause before the first failsafe / loss-of-control.
pub fn rank(events: &[Event]) -> Verdict {
    // anchor = first failsafe or loss of control (crash / tilt / sustained tracking failure)
    let anchor = events.iter().find(|e| (e.role == Role::Response && e.weight >= 0.9)
        || (e.role == Role::Terminal && e.cat == Cat::Crash)
        || (e.role == Role::Symptom && e.cat == Cat::Control)).cloned();
    let horizon = anchor.as_ref().map_or(f64::INFINITY, |a| a.t + 0.5);
    let cands: Vec<&Event> = events.iter().filter(|e| e.cat.causal() && e.t <= horizon && e.role != Role::Terminal).collect();
    // score each category: earliest onset, then evidence strength
    let mut cats: Vec<Cat> = cands.iter().map(|e| e.cat).collect();
    cats.sort(); cats.dedup();
    let t_first = cands.iter().map(|e| e.t).fold(f64::INFINITY, f64::min);
    let mut scored: Vec<(Cat, f64, f64)> = cats.iter().map(|&c| {
        let evs: Vec<&&Event> = cands.iter().filter(|e| e.cat == c).collect();
        let onset = evs.iter().map(|e| e.t).fold(f64::INFINITY, f64::min);
        let strength: f64 = evs.iter().map(|e| e.weight).fold(0.0, f64::max) + 0.1 * (evs.len().min(5) as f64);
        // earlier is better: lose 0.15 per second behind the earliest candidate
        let s = strength - 0.15 * (onset - t_first);
        (c, s, onset)
    }).collect();
    scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    let mut v = Verdict { root: None, root_category: "none".into(), confidence: 0.0, anchor: anchor.clone(), supporting: vec![], chain: vec![], alternatives: vec![] };
    let Some(&(best, bs, _)) = scored.first() else { return v; };
    // a lone weak hint without any failsafe/crash is not reported as a cause
    if anchor.is_none() && bs < 0.85 { return v; }
    let total: f64 = scored.iter().map(|s| s.1.max(0.05)).sum();
    v.confidence = (bs.max(0.05) / total * 100.0).round() / 100.0;
    v.alternatives = scored.iter().skip(1).take(3).map(|s| (s.0.name().to_string(), (s.1.max(0.05) / total * 100.0).round() / 100.0)).collect();
    let root = cands.iter().filter(|e| e.cat == best).min_by(|a, b| a.t.partial_cmp(&b.t).unwrap()).map(|e| (*e).clone());
    v.root_category = best.name().into();
    v.supporting = cands.iter().filter(|e| e.cat == best).take(8).map(|e| (*e).clone()).collect();
    if let Some(r) = &root {
        // chain: first event of each (cat, what-prefix) from root onward, up to the terminal event
        let mut seen: Vec<String> = vec![];
        for e in events.iter().filter(|e| e.t >= r.t) {
            let key = format!("{:?}{}", e.cat, e.what.split(|c: char| c.is_ascii_digit()).next().unwrap_or(""));
            if seen.contains(&key) { continue; }
            seen.push(key);
            v.chain.push(e.clone());
            if e.role == Role::Terminal || v.chain.len() >= 14 { break; }
        }
    }
    v.root = root;
    v
}
