//! Minimal ULog reader following https://docs.px4.io/main/en/dev_log/ulog_file_format.html
//! Handles: header, flag bits (refuses unknown incompat bits), formats (nested types, arrays,
//! omitted trailing padding), info/multi-info, params (incl. changes during flight), defaults,
//! subscriptions, data, logged strings (L and tagged C), dropouts, truncated tail.
use std::collections::HashMap;
use std::fmt;

#[derive(Debug)]
pub enum Error {
    BadMagic,
    Truncated,
    IncompatFlags(u8),
    Format(String),
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::BadMagic => write!(f, "not a ULog file (bad magic)"),
            Error::Truncated => write!(f, "file truncated inside header"),
            Error::IncompatFlags(b) => write!(f, "unknown incompat flag bits 0x{b:02x}; refusing to parse"),
            Error::Format(s) => write!(f, "bad format definition: {s}"),
        }
    }
}
impl std::error::Error for Error {}

const MAGIC: [u8; 7] = [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35];

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Base { I8, U8, I16, U16, I32, U32, I64, U64, F32, F64, Bool, Char }
impl Base {
    fn parse(s: &str) -> Option<Base> {
        Some(match s {
            "int8_t" => Base::I8, "uint8_t" => Base::U8, "int16_t" => Base::I16, "uint16_t" => Base::U16,
            "int32_t" => Base::I32, "uint32_t" => Base::U32, "int64_t" => Base::I64, "uint64_t" => Base::U64,
            "float" => Base::F32, "double" => Base::F64, "bool" => Base::Bool, "char" => Base::Char,
            _ => return None,
        })
    }
    fn size(self) -> usize {
        match self {
            Base::I8 | Base::U8 | Base::Bool | Base::Char => 1,
            Base::I16 | Base::U16 => 2,
            Base::I32 | Base::U32 | Base::F32 => 4,
            Base::I64 | Base::U64 | Base::F64 => 8,
        }
    }
    fn read(self, b: &[u8]) -> f64 {
        macro_rules! le { ($t:ty, $n:expr) => { <$t>::from_le_bytes(b[..$n].try_into().unwrap()) as f64 } }
        match self {
            Base::I8 => b[0] as i8 as f64, Base::U8 | Base::Char => b[0] as f64, Base::Bool => (b[0] != 0) as u8 as f64,
            Base::I16 => le!(i16, 2), Base::U16 => le!(u16, 2), Base::I32 => le!(i32, 4), Base::U32 => le!(u32, 4),
            Base::I64 => le!(i64, 8), Base::U64 => le!(u64, 8), Base::F32 => le!(f32, 4), Base::F64 => le!(f64, 8),
        }
    }
}

#[derive(Clone, Debug)]
struct RawField { ty: String, len: Option<usize>, name: String }

/// Flattened scalar field: "name", "name[3]", "nested.name"
#[derive(Clone, Debug)]
pub struct Field { pub name: String, pub base: Base, pub offset: usize }

#[derive(Debug, Default)]
pub struct Topic {
    pub name: String,
    pub multi_id: u8,
    pub fields: Vec<Field>,
    pub size: usize,
    /// rows of raw bytes (padded to size)
    rows: Vec<Vec<u8>>,
    ts_off: Option<usize>,
}
impl Topic {
    pub fn len(&self) -> usize { self.rows.len() }
    pub fn is_empty(&self) -> bool { self.rows.is_empty() }
    pub fn has(&self, field: &str) -> bool { self.fields.iter().any(|f| f.name == field) }
    pub fn field(&self, name: &str) -> Option<&Field> { self.fields.iter().find(|f| f.name == name) }
    /// timestamps in microseconds
    pub fn ts(&self) -> Vec<u64> {
        match self.ts_off {
            Some(o) => self.rows.iter().map(|r| u64::from_le_bytes(r[o..o + 8].try_into().unwrap())).collect(),
            None => vec![0; self.rows.len()],
        }
    }
    /// column as f64; None if field missing
    pub fn col(&self, name: &str) -> Option<Vec<f64>> {
        let f = self.field(name)?;
        let n = f.base.size();
        Some(self.rows.iter().map(|r| f.base.read(&r[f.offset..f.offset + n])).collect())
    }
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct LogMsg { pub ts: u64, pub level: u8, pub text: String }

#[derive(Debug, Clone, Copy)]
pub struct ParamChange { pub ts: u64, pub value: f64 }

#[derive(Debug, Default)]
pub struct ULog {
    pub start_us: u64,
    pub version: u8,
    pub info: HashMap<String, String>,
    pub params: HashMap<String, f64>,
    pub param_changes: HashMap<String, Vec<ParamChange>>,
    pub topics: Vec<Topic>,
    pub messages: Vec<LogMsg>,
    pub dropouts: Vec<(u64, u16)>,
    pub truncated: bool,
    pub last_ts: u64,
}

fn flatten(formats: &HashMap<String, Vec<RawField>>, name: &str, prefix: &str, base_off: usize,
           out: &mut Vec<Field>, depth: usize) -> Result<usize, Error> {
    if depth > 16 { return Err(Error::Format(format!("nesting too deep at {name}"))); }
    let fields = formats.get(name).ok_or_else(|| Error::Format(format!("unknown type {name}")))?;
    let mut off = base_off;
    for f in fields {
        let count = f.len.unwrap_or(1);
        if let Some(b) = Base::parse(&f.ty) {
            let pad = f.name.starts_with("_padding");
            for i in 0..count {
                if !pad && !(b == Base::Char && f.len.is_some()) {
                    let n = if f.len.is_some() { format!("{prefix}{}[{i}]", f.name) } else { format!("{prefix}{}", f.name) };
                    out.push(Field { name: n, base: b, offset: off });
                }
                off += b.size();
            }
        } else {
            for i in 0..count {
                let p = if f.len.is_some() { format!("{prefix}{}[{i}].", f.name) } else { format!("{prefix}{}.", f.name) };
                off = flatten(formats, &f.ty, &p, off, out, depth + 1)?;
            }
        }
    }
    Ok(off)
}

fn read_key_value(key: &str, val: &[u8]) -> (String, String, Option<f64>) {
    let (ty, name) = key.split_once(' ').unwrap_or(("", key));
    let num = match ty {
        "int32_t" if val.len() >= 4 => Some(i32::from_le_bytes(val[..4].try_into().unwrap()) as f64),
        "float" if val.len() >= 4 => Some(f32::from_le_bytes(val[..4].try_into().unwrap()) as f64),
        "uint32_t" if val.len() >= 4 => Some(u32::from_le_bytes(val[..4].try_into().unwrap()) as f64),
        _ => None,
    };
    let s = match num { Some(n) => format!("{n}"), None => String::from_utf8_lossy(val).trim_end_matches('\0').to_string() };
    (name.to_string(), s, num)
}

impl ULog {
    pub fn parse(buf: &[u8]) -> Result<ULog, Error> {
        if buf.len() < 16 { return Err(Error::Truncated); }
        if buf[..7] != MAGIC { return Err(Error::BadMagic); }
        let mut u = ULog { version: buf[7], start_us: u64::from_le_bytes(buf[8..16].try_into().unwrap()), ..Default::default() };
        let mut formats: HashMap<String, Vec<RawField>> = HashMap::new();
        let mut subs: HashMap<u16, usize> = HashMap::new();
        let mut in_data = false;
        let mut pos = 16;
        let mut appended: Vec<u64> = vec![];
        while pos + 3 <= buf.len() {
            let size = u16::from_le_bytes([buf[pos], buf[pos + 1]]) as usize;
            let ty = buf[pos + 2];
            let body_start = pos + 3;
            if let Some(&o) = appended.first() {
                let o = o as usize;
                if o > pos && o < body_start + size { pos = o; appended.remove(0); continue; }
            }
            if body_start + size > buf.len() { u.truncated = true; break; }
            let b = &buf[body_start..body_start + size];
            pos = body_start + size;
            match ty {
                b'B' if b.len() >= 40 => {
                    let inc = b[8];
                    if inc & !1 != 0 || b[9..16].iter().any(|&x| x != 0) { return Err(Error::IncompatFlags(inc)); }
                    for k in 0..3 {
                        let o = u64::from_le_bytes(b[16 + 8 * k..24 + 8 * k].try_into().unwrap());
                        if o != 0 { appended.push(o); }
                    }
                }
                b'F' => {
                    let s = String::from_utf8_lossy(b);
                    let (name, rest) = s.split_once(':').ok_or_else(|| Error::Format(s.to_string()))?;
                    let mut fields = vec![];
                    for fd in rest.split(';').filter(|x| !x.is_empty()) {
                        let (t, n) = fd.trim().split_once(' ').ok_or_else(|| Error::Format(fd.to_string()))?;
                        let (t, len) = match t.split_once('[') {
                            Some((t, l)) => (t.to_string(), Some(l.trim_end_matches(']').parse().map_err(|_| Error::Format(fd.to_string()))?)),
                            None => (t.to_string(), None),
                        };
                        fields.push(RawField { ty: t, len, name: n.to_string() });
                    }
                    formats.insert(name.to_string(), fields);
                }
                b'I' | b'P' if !b.is_empty() => {
                    let kl = b[0] as usize;
                    if 1 + kl > b.len() { continue; }
                    let key = String::from_utf8_lossy(&b[1..1 + kl]).to_string();
                    let (name, s, num) = read_key_value(&key, &b[1 + kl..]);
                    if ty == b'I' { u.info.insert(name, s); } else if let Some(v) = num {
                        if in_data {
                            u.param_changes.entry(name.clone()).or_default().push(ParamChange { ts: u.last_ts, value: v });
                        } else { u.params.insert(name, v); }
                    }
                }
                b'M' if b.len() >= 2 => {
                    let kl = b[1] as usize;
                    if 2 + kl > b.len() { continue; }
                    let key = String::from_utf8_lossy(&b[2..2 + kl]).to_string();
                    let (name, s, _) = read_key_value(&key, &b[2 + kl..]);
                    u.info.entry(name).and_modify(|e| e.push_str(&s)).or_insert(s);
                }
                b'A' if b.len() >= 3 => {
                    in_data = true;
                    let id = u16::from_le_bytes([b[1], b[2]]);
                    let (multi, name) = (b[0], String::from_utf8_lossy(&b[3..]).to_string());
                    let mut fields = vec![];
                    let size = flatten(&formats, &name, "", 0, &mut fields, 0)?;
                    let ts_off = fields.iter().find(|f| f.name == "timestamp" && f.base == Base::U64).map(|f| f.offset);
                    u.topics.push(Topic { name, multi_id: multi, fields, size, rows: vec![], ts_off });
                    subs.insert(id, u.topics.len() - 1);
                }
                b'D' if b.len() >= 2 => {
                    let id = u16::from_le_bytes([b[0], b[1]]);
                    if let Some(&ti) = subs.get(&id) {
                        let t = &mut u.topics[ti];
                        let mut row = b[2..].to_vec();
                        if row.len() > t.size { row.truncate(t.size); }
                        row.resize(t.size, 0); // omitted trailing padding
                        if let Some(o) = t.ts_off { if o + 8 <= row.len() {
                            let ts = u64::from_le_bytes(row[o..o + 8].try_into().unwrap());
                            if ts > u.last_ts { u.last_ts = ts; }
                        } }
                        t.rows.push(row);
                    }
                }
                b'L' if b.len() >= 9 => {
                    in_data = true;
                    let ts = u64::from_le_bytes(b[1..9].try_into().unwrap());
                    u.messages.push(LogMsg { ts, level: b[0].saturating_sub(b'0'), text: String::from_utf8_lossy(&b[9..]).trim().to_string() });
                }
                b'C' if b.len() >= 11 => {
                    let ts = u64::from_le_bytes(b[3..11].try_into().unwrap());
                    u.messages.push(LogMsg { ts, level: b[0].saturating_sub(b'0'), text: String::from_utf8_lossy(&b[11..]).trim().to_string() });
                }
                b'O' if b.len() >= 2 => u.dropouts.push((u.last_ts, u16::from_le_bytes([b[0], b[1]]))),
                b'R' if b.len() >= 2 => { subs.remove(&u16::from_le_bytes([b[0], b[1]])); }
                _ => {} // unknown / S / Q: ignore
            }
            if let Some(&o) = appended.first() { if pos as u64 >= o { appended.remove(0); } }
        }
        Ok(u)
    }
    pub fn topic(&self, name: &str) -> Option<&Topic> {
        self.topics.iter().filter(|t| t.name == name && !t.is_empty()).min_by_key(|t| t.multi_id)
    }
    pub fn topics_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Topic> + 'a {
        self.topics.iter().filter(move |t| t.name == name && !t.is_empty())
    }
}

/// Write a smaller ULog: keep only data of `keep` topics, at most one sample per `min_dt_us`
/// per subscription. Definitions, params, strings are copied verbatim. Output is a valid ULog.
pub fn trim(buf: &[u8], keep: &[&str], min_dt_us: u64) -> Result<Vec<u8>, Error> {
    if buf.len() < 16 || buf[..7] != MAGIC { return Err(Error::BadMagic); }
    let mut out = buf[..16].to_vec();
    let mut subs: HashMap<u16, (bool, Option<u64>)> = HashMap::new();
    let mut pos = 16;
    while pos + 3 <= buf.len() {
        let size = u16::from_le_bytes([buf[pos], buf[pos + 1]]) as usize;
        let ty = buf[pos + 2];
        let end = pos + 3 + size;
        if end > buf.len() { break; }
        let b = &buf[pos + 3..end];
        let mut copy = true;
        match ty {
            b'B' => {
                // clear appended offsets: trimmed output is contiguous
                let mut m = buf[pos..end].to_vec();
                if m.len() >= 3 + 40 { for x in &mut m[3 + 8..3 + 40] { *x = 0; } }
                out.extend_from_slice(&m); copy = false;
            }
            b'A' if b.len() >= 3 => {
                let name = String::from_utf8_lossy(&b[3..]).to_string();
                subs.insert(u16::from_le_bytes([b[1], b[2]]), (keep.contains(&name.as_str()), None));
            }
            b'D' if b.len() >= 10 => {
                let id = u16::from_le_bytes([b[0], b[1]]);
                let ts = u64::from_le_bytes(b[2..10].try_into().unwrap());
                match subs.get_mut(&id) {
                    Some((true, last)) if last.is_none_or(|l| ts >= l + min_dt_us || ts < l) => { *last = Some(ts); }
                    _ => copy = false,
                }
            }
            b'S' => copy = false,
            _ => {}
        }
        if copy { out.extend_from_slice(&buf[pos..end]); }
        pos = end;
    }
    Ok(out)
}
