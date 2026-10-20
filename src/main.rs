use std::process::ExitCode;
use ulog_why::{report, ulog};

const USAGE: &str = "ulog-why: find the earliest likely causal failure in a PX4 ULog (.ulg)

USAGE:
  ulog-why <log.ulg> [--json] [--timeline]   analyze a log
  ulog-why demo                              analyze the bundled demo log (offline)
  ulog-why topics <log.ulg>                  list logged topics and sample counts
  ulog-why trim <in.ulg> <out.ulg> [hz]      write a smaller ULog with only the topics ulog-why reads
";

const DEMO: &[u8] = include_bytes!("../examples/demo_gps_loss.ulg");

const KEEP: &[&str] = &["vehicle_status", "vehicle_land_detected", "vehicle_gps_position", "sensor_gps", "vehicle_air_data",
    "vehicle_magnetometer", "estimator_innovation_test_ratios", "estimator_status_flags", "battery_status", "actuator_motors",
    "failure_detector_status", "vehicle_local_position", "vehicle_local_position_setpoint", "vehicle_attitude",
    "vehicle_attitude_setpoint", "failsafe_flags"];

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let json = args.iter().any(|a| a == "--json");
    let full = args.iter().any(|a| a == "--timeline");
    let pos: Vec<&String> = args.iter().filter(|a| !a.starts_with("--")).collect();
    let fail = |m: String| { eprintln!("error: {m}"); ExitCode::from(2) };
    match pos.first().map(|s| s.as_str()) {
        None => { print!("{USAGE}"); ExitCode::from(1) }
        Some("-h") | Some("help") => { print!("{USAGE}"); ExitCode::SUCCESS }
        Some("topics") => {
            let Some(p) = pos.get(1) else { return fail("missing file".into()) };
            let buf = match std::fs::read(p) { Ok(b) => b, Err(e) => return fail(format!("{p}: {e}")) };
            match ulog::ULog::parse(&buf) {
                Ok(u) => { for t in u.topics.iter().filter(|t| !t.is_empty()) { println!("{:<45} {:>2} {:>8}", t.name, t.multi_id, t.len()); } ExitCode::SUCCESS }
                Err(e) => fail(e.to_string()),
            }
        }
        Some("trim") => {
            let (Some(i), Some(o)) = (pos.get(1), pos.get(2)) else { return fail("usage: trim <in> <out> [hz]".into()) };
            let hz: f64 = pos.get(3).and_then(|h| h.parse().ok()).unwrap_or(10.0);
            let buf = match std::fs::read(i) { Ok(b) => b, Err(e) => return fail(format!("{i}: {e}")) };
            match ulog::trim(&buf, KEEP, (1e6 / hz) as u64) {
                Ok(b) => { if let Err(e) = std::fs::write(o, &b) { return fail(e.to_string()); } eprintln!("{} -> {} bytes", buf.len(), b.len()); ExitCode::SUCCESS }
                Err(e) => fail(e.to_string()),
            }
        }
        Some(p) => {
            let (name, buf) = if p == "demo" { ("demo_gps_loss.ulg (bundled)".to_string(), DEMO.to_vec()) } else {
                match std::fs::read(p) { Ok(b) => (p.to_string(), b), Err(e) => return fail(format!("{p}: {e}")) }
            };
            match report::run(&name, &buf) {
                Ok(r) => {
                    if json { println!("{}", serde_json::to_string_pretty(&r).unwrap()); } else { print!("{}", report::text(&r, full)); }
                    ExitCode::SUCCESS
                }
                Err(e) => fail(e.to_string()),
            }
        }
    }
}
