use ulog_why::analyze::{self, Cat};
use ulog_why::ulog::{trim, Error, ULog};

fn msg(out: &mut Vec<u8>, ty: u8, body: &[u8]) {
    out.extend_from_slice(&(body.len() as u16).to_le_bytes());
    out.push(ty);
    out.extend_from_slice(body);
}
fn header(incompat: u8) -> Vec<u8> {
    let mut b = vec![0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35, 0x01];
    b.extend_from_slice(&123u64.to_le_bytes());
    let mut fb = vec![0u8; 40];
    fb[8] = incompat;
    msg(&mut b, b'B', &fb);
    b
}
fn kv(key: &str, val: &[u8]) -> Vec<u8> {
    let mut v = vec![key.len() as u8];
    v.extend_from_slice(key.as_bytes());
    v.extend_from_slice(val);
    v
}

fn sample_log() -> Vec<u8> {
    let mut b = header(0);
    msg(&mut b, b'I', &kv("char[3] sys_name", b"PX4"));
    msg(&mut b, b'P', &kv("float MPC_XY_VEL_MAX", &12.0f32.to_le_bytes()));
    // nested type defined after use, array, trailing padding
    msg(&mut b, b'F', b"outer:uint64_t timestamp;inner[2] pair;uint8_t flag;uint8_t[3] _padding0;");
    msg(&mut b, b'F', b"inner:float a;int16_t b;");
    let mut a = vec![0u8]; a.extend_from_slice(&0u16.to_le_bytes()); a.extend_from_slice(b"outer");
    msg(&mut b, b'A', &a);
    for k in 0..3u64 {
        let mut d = 0u16.to_le_bytes().to_vec();
        d.extend_from_slice(&(1_000_000 * k).to_le_bytes());
        for j in 0..2 { d.extend_from_slice(&(k as f32 + j as f32 * 0.5).to_le_bytes()); d.extend_from_slice(&(-(k as i16)).to_le_bytes()); }
        d.push(k as u8 % 2); // padding omitted on purpose
        msg(&mut b, b'D', &d);
    }
    let mut l = vec![b'4']; l.extend_from_slice(&1_500_000u64.to_le_bytes()); l.extend_from_slice(b"[commander] GPS lost");
    msg(&mut b, b'L', &l);
    msg(&mut b, b'P', &kv("float MPC_XY_VEL_MAX", &5.0f32.to_le_bytes()));
    msg(&mut b, b'O', &250u16.to_le_bytes());
    b
}

#[test]
fn parses_nested_arrays_padding_strings_params() {
    let u = ULog::parse(&sample_log()).unwrap();
    assert_eq!(u.start_us, 123);
    assert_eq!(u.info["sys_name"], "PX4");
    assert_eq!(u.params["MPC_XY_VEL_MAX"], 12.0);
    assert_eq!(u.param_changes["MPC_XY_VEL_MAX"][0].value, 5.0);
    let t = u.topic("outer").unwrap();
    assert_eq!(t.size, 8 + 2 * 6 + 1 + 3);
    assert_eq!(t.len(), 3);
    assert_eq!(t.ts(), vec![0, 1_000_000, 2_000_000]);
    assert_eq!(t.col("pair[1].a").unwrap(), vec![0.5, 1.5, 2.5]);
    assert_eq!(t.col("pair[0].b").unwrap(), vec![0.0, -1.0, -2.0]);
    assert_eq!(t.col("flag").unwrap(), vec![0.0, 1.0, 0.0]);
    assert!(!t.has("_padding0[0]"));
    assert_eq!(u.messages[0].level, 4);
    assert_eq!(u.messages[0].text, "[commander] GPS lost");
    assert_eq!(u.dropouts, vec![(2_000_000, 250)]);
    assert!(!u.truncated);
}

#[test]
fn truncated_tail_is_discarded() {
    let mut b = sample_log();
    let full = ULog::parse(&b).unwrap().topic("outer").unwrap().len();
    let mut d = 0u16.to_le_bytes().to_vec(); d.extend_from_slice(&[1, 2, 3, 4, 5]);
    msg(&mut b, b'D', &d);
    b.truncate(b.len() - 3);
    let u = ULog::parse(&b).unwrap();
    assert!(u.truncated);
    assert_eq!(u.topic("outer").unwrap().len(), full);
}

#[test]
fn refuses_unknown_incompat_bits_and_bad_magic() {
    assert!(matches!(ULog::parse(&header(0x02)), Err(Error::IncompatFlags(2))));
    assert!(matches!(ULog::parse(b"not a ulog file at all"), Err(Error::BadMagic)));
    assert!(ULog::parse(&header(0x01)).is_ok()); // DATA_APPENDED is known
}

#[test]
fn unknown_message_types_are_ignored() {
    let mut b = sample_log();
    msg(&mut b, b'Z', &[1, 2, 3]);
    assert!(ULog::parse(&b).is_ok());
}

#[test]
fn trim_keeps_valid_log() {
    let b = sample_log();
    let t = trim(&b, &["outer"], 1_500_000).unwrap();
    let u = ULog::parse(&t).unwrap();
    assert_eq!(u.topic("outer").unwrap().ts(), vec![0, 2_000_000]);
    let none = trim(&b, &[], 0).unwrap();
    assert!(ULog::parse(&none).unwrap().topic("outer").is_none());
}

#[test]
fn demo_log_finds_gps() {
    let buf = include_bytes!("../examples/demo_gps_loss.ulg");
    let u = ULog::parse(buf).unwrap();
    let f = analyze::flight(&u);
    let ev = analyze::detect(&u, &f);
    let v = analyze::rank(&ev);
    assert_eq!(v.root_category, Cat::Gps.name());
    assert!(v.anchor.is_some(), "demo should end in a failsafe");
    assert!(v.root.unwrap().t <= v.anchor.unwrap().t);
}
