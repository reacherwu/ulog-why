#!/usr/bin/env python3
"""Generate labeled PX4 SITL (SIH quad) ulogs with one injected fault each.

Each trial: fresh PX4 instance -> arm, take off to 15-25 m, reposition to a random point,
inject one fault at a random time, keep flying for up to 60 s, stop. Writes
<out>/<trial>.ulg and appends the ground-truth label to <out>/labels.jsonl.
The injection time is logged as wall-clock offset and as PX4 boot time (from SYSTEM_TIME / ATTITUDE time_boot_ms).
"""
import argparse, json, math, os, random, shutil, signal, subprocess, sys, time, glob
from pymavlink import mavutil

FAULTS = ["gps_loss", "baro_offset", "mag_offset", "thrust_loss", "battery_fail", "datalink_loss", "none"]

def start_px4(px4, inst):
    build = os.path.join(px4, "build/px4_sitl_default")
    wd = os.path.join(build, f"uwhy_{inst}")
    shutil.rmtree(wd, ignore_errors=True)
    os.makedirs(wd)
    env = dict(os.environ, HEADLESS="1", PX4_SIM_MODEL="sihsim_quadx", PX4_SYS_AUTOSTART="10040")
    p = subprocess.Popen([os.path.join(build, "bin/px4"), "-i", str(inst), "-d", os.path.join(build, "etc")],
                         cwd=wd, env=env, stdout=open(os.path.join(wd, "px4.out"), "w"), stderr=subprocess.STDOUT,
                         start_new_session=True)
    return p, wd

class V:
    def __init__(self, inst):
        self.m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{14540 + inst}", source_system=255, source_component=190)
        self.m.wait_heartbeat(timeout=60)
        self.ts = self.m.target_system
        self.hb = True; self.last_hb = 0; self.boot_ms = 0; self.armed = False
        self.gpos = None; self.texts = []; self.relalt = 0
    def poll(self, dur=0.0):
        end = time.time() + dur
        while True:
            now = time.time()
            if self.hb and now - self.last_hb > 0.5:
                self.m.mav.heartbeat_send(6, 8, 0, 0, 0); self.last_hb = now
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                t = msg.get_type()
                if hasattr(msg, "time_boot_ms"): self.boot_ms = max(self.boot_ms, msg.time_boot_ms)
                if t == "HEARTBEAT" and msg.get_srcComponent() == 1: self.armed = bool(msg.base_mode & 128)
                elif t == "GLOBAL_POSITION_INT": self.gpos = (msg.lat, msg.lon, msg.alt); self.relalt = msg.relative_alt / 1000
                elif t == "STATUSTEXT": self.texts.append((self.boot_ms, msg.text))
            if time.time() >= end: return
    def cmd(self, c, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(self.ts, 1, c, 0, *p)
    def param(self, name, val, int_=False):
        ty = mavutil.mavlink.MAV_PARAM_TYPE_INT32 if int_ else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        if int_:
            import struct
            val = struct.unpack("<f", struct.pack("<i", int(val)))[0]
        for _ in range(3):
            self.m.mav.param_set_send(self.ts, 1, name.encode(), float(val), ty); self.poll(0.1)

def trial(args, inst, tid, fault, rng):
    p, wd = start_px4(args.px4, inst)
    lab = dict(trial=tid, fault=fault, instance=inst)
    v = None
    try:
        v = V(inst)
        v.poll(8)
        v.param("SIM_BAT_MIN_PCT", 100 if fault != "battery_fail" else 60)
        v.param("NAV_DLL_ACT", 2, True); v.param("COM_DL_LOSS_T", 5, True)
        v.param("SYS_FAILURE_EN", 1, True)
        v.poll(1)
        alt = rng.uniform(15, 25)
        t0 = time.time()
        while not v.armed:
            v.cmd(400, 1); v.poll(1.0)
            if time.time() - t0 > 60: raise RuntimeError("arm timeout")
        v.cmd(22, 0, 0, 0, float("nan"), float("nan"), float("nan"), (v.gpos[2] / 1000) + alt)
        v.poll(rng.uniform(12, 16))
        lat, lon, a = v.gpos
        d, b = rng.uniform(40, 120), rng.uniform(0, 2 * math.pi)
        tlat = lat + int(d * math.cos(b) / 111111 * 1e7); tlon = lon + int(d * math.sin(b) / (111111 * math.cos(math.radians(lat / 1e7))) * 1e7)
        v.m.mav.command_int_send(v.ts, 1, 0, 192, 0, 0, -1, 1, 0, float("nan"), tlat, tlon, a / 1000)
        v.poll(rng.uniform(4, 18))
        lab["inject_boot_ms"] = v.boot_ms
        if fault == "gps_loss": v.param("SIM_GPS_USED", 0, True); lab["mag"] = 0
        elif fault == "baro_offset":
            m = rng.choice([-1, 1]) * rng.uniform(300, 2000); v.param("SIM_BARO_OFF_P", m); lab["mag"] = m
        elif fault == "mag_offset":
            m = rng.choice([-1, 1]) * rng.uniform(0.4, 1.5); v.param("SIM_MAG_OFFSET_X", m); lab["mag"] = m
        elif fault == "thrust_loss":
            m = rng.uniform(1.6, 2.7); v.param("SIH_T_MAX", m); lab["mag"] = m
        elif fault == "battery_fail": v.cmd(420, 100, 1, 0); lab["mag"] = 0  # FAILURE_UNIT_SYSTEM_BATTERY=100, FAILURE_TYPE_OFF=1
        elif fault == "datalink_loss": v.hb = False; lab["mag"] = 0
        t_end = time.time() + args.after
        while time.time() < t_end:
            v.poll(0.5)
            if not v.armed and time.time() > t_end - args.after + 3: break
        lab["armed_at_end"] = v.armed
        lab["statustext"] = [t for t in v.texts if t[0] >= lab["inject_boot_ms"] - 2000][:30]
        v.hb = True; v.poll(0.5)
        v.cmd(400, 0, 21196); v.poll(2)  # force disarm -> logger closes file
    except Exception as e:
        lab["error"] = repr(e)
    finally:
        try: v.m.close()  # a leaked udpin socket on the same port starves the next trial
        except Exception: pass
        os.killpg(p.pid, signal.SIGINT); time.sleep(2)
        try: os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError: pass
    logs = sorted(glob.glob(os.path.join(wd, "log", "*", "*.ulg")), key=os.path.getmtime)
    if logs:
        shutil.move(logs[-1], os.path.join(args.out, f"{tid}.ulg")); lab["ulg"] = f"{tid}.ulg"
    shutil.rmtree(wd, ignore_errors=True)
    return lab

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--px4", default="/workspace/px4/PX4-Autopilot")
    ap.add_argument("--out", required=True); ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--seed", type=int, default=1); ap.add_argument("--per-fault", type=int, default=1)
    ap.add_argument("--faults", default=",".join(FAULTS)); ap.add_argument("--after", type=float, default=45)
    ap.add_argument("--prefix", default="t")
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    rng = random.Random(a.seed)
    plan = [f for f in a.faults.split(",") for _ in range(a.per_fault)]; rng.shuffle(plan)
    for k, f in enumerate(plan):
        tid = f"{a.prefix}{a.seed}_{k:02d}"
        lab = trial(a, a.instance, tid, f, rng); lab["seed"] = a.seed
        print(json.dumps({k2: lab.get(k2) for k2 in ("trial", "fault", "inject_boot_ms", "armed_at_end", "error")}), flush=True)
        with open(os.path.join(a.out, "labels.jsonl"), "a") as fh: fh.write(json.dumps(lab) + "\n")
