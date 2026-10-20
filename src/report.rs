use crate::analyze::{self, Event, Role, Verdict};
use crate::ulog::ULog;
use serde::Serialize;

#[derive(Serialize)]
pub struct Report {
    pub file: String,
    pub px4_version: Option<String>,
    pub arm_s: Option<f64>,
    pub disarm_s: Option<f64>,
    pub log_end_s: f64,
    pub truncated: bool,
    pub verdict: Verdict,
    pub timeline: Vec<Event>,
}

pub fn run(file: &str, buf: &[u8]) -> Result<Report, crate::ulog::Error> {
    let u = ULog::parse(buf)?;
    let f = analyze::flight(&u);
    let events = analyze::detect(&u, &f);
    let verdict = analyze::rank(&events);
    Ok(Report {
        file: file.to_string(),
        px4_version: u.info.get("ver_sw_release").map(|v| {
            let x = v.parse::<f64>().unwrap_or(0.0) as u32;
            format!("v{}.{}.{}", x >> 24, (x >> 16) & 0xff, (x >> 8) & 0xff)
        }),
        arm_s: f.arm, disarm_s: f.disarm, log_end_s: f.end, truncated: u.truncated, verdict, timeline: events,
    })
}

fn rel(t: f64, arm: Option<f64>) -> String {
    match arm { Some(a) => format!("T{:+7.2}s", t - a), None => format!("{t:8.2}s") }
}
fn role(r: Role) -> &'static str {
    match r { Role::Cause => "cause?", Role::Symptom => "symptom", Role::Response => "response", Role::Terminal => "end" }
}

pub fn text(r: &Report, full: bool) -> String {
    let mut s = String::new();
    let v = &r.verdict;
    s += &format!("ulog-why  {}  ({})\n", r.file, r.px4_version.clone().unwrap_or("unknown PX4".into()));
    s += &format!("times are relative to arming (T+0){}\n\n", if r.truncated { "; log ends mid-message (truncated)" } else { "" });
    match &v.root {
        None => {
            s += "No causal anomaly found";
            s += if v.anchor.is_some() { " before the failsafe.\n" } else { " (no failsafe, loss of control or strong sensor fault in this log).\n" };
        }
        Some(root) => {
            s += &format!("EARLIEST LIKELY CAUSE: {}  (confidence {:.0}%)\n", v.root_category.to_uppercase(), v.confidence * 100.0);
            s += &format!("  {}  {}   [{}]\n", rel(root.t, r.arm_s), root.what, root.source);
            if let Some(a) = &v.anchor {
                s += &format!("  first failsafe / loss of control: {}  {}  ({:.1} s after the cause)\n", rel(a.t, r.arm_s), a.what, a.t - root.t);
            }
            if v.supporting.len() > 1 {
                s += "\nEvidence:\n";
                for e in &v.supporting { s += &format!("  {}  {}   [{}]\n", rel(e.t, r.arm_s), e.what, e.source); }
            }
            s += "\nChain (cause -> outcome):\n";
            for (i, e) in v.chain.iter().enumerate() {
                s += &format!("  {:>2}. {}  {:<10} {:<8} {}\n", i + 1, rel(e.t, r.arm_s), e.cat.name(), role(e.role), e.what);
            }
            if !v.alternatives.is_empty() {
                s += "\nOther candidates: ";
                s += &v.alternatives.iter().map(|(c, p)| format!("{c} {:.0}%", p * 100.0)).collect::<Vec<_>>().join(", ");
                s += "\n";
            }
        }
    }
    if full {
        s += &format!("\nFull timeline ({} events):\n", r.timeline.len());
        for e in &r.timeline { s += &format!("  {}  {:<10} {:<8} {}\n", rel(e.t, r.arm_s), e.cat.name(), role(e.role), e.what); }
    }
    s
}
