#!/usr/bin/env python3
"""TVC Tuner — local web UI for tuning the rocket's TVC to its physical design.

Runs a physics simulation of YOUR rocket (mass, center of mass, moment of
inertia, motor thrust, TVC geometry, servo dynamics, aerodynamics) with an
exact software copy of the firmware's PID controller, so the gains you see
behaving well here behave the same on the board.

Buttons in the UI can:
  * suggest PID gains from the rocket's physics (pole placement),
  * write the chosen parameters into firmware/TVC_FlightComputer/config.h,
  * compile + upload the firmware to a connected Teensy (via arduino-cli),
  * push gains live over USB serial to a running board ("set"/"save").

Usage:
    pip install -r requirements.txt
    python app.py            # then open http://127.0.0.1:5055
"""
from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
import time
from pathlib import Path

from flask import Flask, Response, jsonify, render_template, request, stream_with_context

ROOT = Path(__file__).resolve().parents[2]
SKETCH_DIR = ROOT / "firmware" / "TVC_FlightComputer"
CONFIG_H = SKETCH_DIR / "config.h"
FQBN = "teensy:avr:teensy40"

RHO_AIR = 1.225  # kg/m^3
G = 9.80665

app = Flask(__name__)


# ---------------------------------------------------------------------------
# Mass properties
# ---------------------------------------------------------------------------
def mass_properties(p: dict) -> dict:
    """Total mass [kg], CM from nozzle [m], pitch inertia about CM [kg m^2].

    Detailed mode sums a component table (each entry: mass in grams,
    position of its own CM above the nozzle in cm, and optional length in
    cm — items with a length are treated as slender rods, m*l^2/12).
    """
    if p.get("mass_mode") == "detailed":
        comps = [c for c in p.get("components", []) if float(c.get("mass_g", 0)) > 0]
        if not comps:
            raise ValueError("Component table is empty.")
        m = sum(float(c["mass_g"]) for c in comps) / 1000.0
        cm = sum(float(c["mass_g"]) / 1000.0 * float(c["pos_cm"]) / 100.0 for c in comps) / m
        inertia = 0.0
        for c in comps:
            mi = float(c["mass_g"]) / 1000.0
            xi = float(c["pos_cm"]) / 100.0
            li = float(c.get("len_cm", 0) or 0) / 100.0
            inertia += mi * li * li / 12.0 + mi * (xi - cm) ** 2
    else:
        m = float(p["total_mass_g"]) / 1000.0
        cm = float(p["cm_from_nozzle_cm"]) / 100.0
        if p.get("inertia_gcm2"):
            inertia = float(p["inertia_gcm2"]) * 1e-7  # g cm^2 -> kg m^2
        else:
            length = float(p.get("rocket_length_cm", 60)) / 100.0
            inertia = m * length * length / 12.0  # uniform-rod estimate
    if m <= 0 or cm <= 0 or inertia <= 0:
        raise ValueError("Mass, CM and inertia must all be positive.")
    return {"mass_kg": m, "cm_m": cm, "inertia_kgm2": inertia}


# ---------------------------------------------------------------------------
# Flight simulation (planar 3-DOF: tilt + vertical + lateral)
# ---------------------------------------------------------------------------
def simulate(p: dict) -> dict:
    mp_ = mass_properties(p)
    m0, cm, i0 = mp_["mass_kg"], mp_["cm_m"], mp_["inertia_kgm2"]

    thrust = float(p["thrust_n"])
    burn = float(p["burn_s"])
    prop_m = float(p.get("prop_mass_g", 0)) / 1000.0
    diameter = float(p.get("body_diameter_cm", 7.5)) / 100.0
    area = math.pi * (diameter / 2) ** 2

    gimbal_lim = math.radians(float(p["gimbal_limit_deg"]))
    gear = float(p["gear_ratio"])
    servo_spd = float(p.get("servo_speed_s_per_60deg", 0.12))
    servo_rate = math.radians(60.0 / max(servo_spd, 1e-3)) / max(gear, 1e-3)
    fw_rate_lim = math.radians(float(p.get("nozzle_rate_limit_dps", 0)) or 1e9)
    nozzle_rate = min(servo_rate, fw_rate_lim)  # rad/s the nozzle can actually do
    latency = float(p.get("servo_latency_ms", 20)) / 1000.0

    kp, ki, kd = float(p["kp"]), float(p["ki"]), float(p["kd"])
    i_lim = math.radians(float(p.get("i_limit", 3)))
    lpf_hz = float(p.get("dterm_lpf_hz", 30))
    ctrl_dt = 1.0 / float(p.get("control_rate_hz", 250))
    tvc_delay = float(p.get("tvc_delay_ms", 150)) / 1000.0

    aero_on = bool(p.get("aero_enabled", False))
    cp = float(p.get("cp_from_nozzle_cm", 0)) / 100.0
    cna = float(p.get("cna", 2.0))
    cd = float(p.get("cd", 0.6))
    wind = float(p.get("wind_ms", 0))

    theta = math.radians(float(p.get("init_tilt_deg", 3)))
    omega = math.radians(float(p.get("init_rate_dps", 0)))
    misalign = math.radians(float(p.get("misalign_deg", 0)))

    dt = 0.0005
    t_end = burn + 0.3
    t = 0.0
    v_vert, v_lat, drift, alt = 0.0, 0.0, 0.0, 0.0
    gimbal = 0.0            # actual nozzle angle (rad)
    integral = 0.0
    rate_filt = 0.0
    next_ctrl = 0.0
    cmd = 0.0               # controller output (rad)
    delayed_target = 0.0    # command after transport delay (what servo chases)
    cmd_queue: list[tuple[float, float]] = []  # (time issued, value) for latency
    sat_steps = 0
    ctrl_steps = 0

    out = {k: [] for k in ("t", "tilt", "gimbal", "cmd", "alt", "drift")}
    rec_every = max(1, int(t_end / dt / 1200))
    step = 0
    max_tilt = abs(math.degrees(theta))
    settle_t = None

    while t < t_end:
        burning = t < burn
        thr = thrust if burning else 0.0
        frac = min(t / burn, 1.0) if burn > 0 else 1.0
        m = m0 - prop_m * frac
        inertia = i0 * (m / m0)  # first-order: inertia shrinks with mass

        # ---- controller (exact copy of firmware AxisPID) ----
        if t >= next_ctrl:
            next_ctrl += ctrl_dt
            ctrl_steps += 1
            if burning and t >= tvc_delay:
                err = -theta
                if lpf_hz > 0:
                    alpha = ctrl_dt / (ctrl_dt + 1.0 / (2 * math.pi * lpf_hz))
                    rate_filt += alpha * (omega - rate_filt)
                else:
                    rate_filt = omega
                integral += ki * err * ctrl_dt
                integral = max(-i_lim, min(i_lim, integral))
                raw = kp * err + integral - kd * rate_filt
                cmd = max(-gimbal_lim, min(gimbal_lim, raw))
                if abs(cmd) >= gimbal_lim * 0.99:
                    sat_steps += 1
            else:
                cmd = 0.0
            cmd_queue.append((t, cmd))

        # ---- actuator: transport delay + slew-rate limit ----
        while cmd_queue and cmd_queue[0][0] <= t - latency:
            delayed_target = cmd_queue.pop(0)[1]
        step_lim = nozzle_rate * dt
        gimbal += max(-step_lim, min(step_lim, delayed_target - gimbal))

        # ---- rotational dynamics ----
        torque = 0.0
        if burning:
            torque += thr * math.sin(gimbal) * cm          # TVC control torque
            torque += thr * math.sin(misalign) * cm        # thrust misalignment
        if aero_on and cp > 0:
            v_air_lat = v_lat - wind
            speed2 = v_vert * v_vert + v_air_lat * v_air_lat
            if speed2 > 0.01:
                gamma = math.atan2(v_air_lat, max(v_vert, 0.1))
                alpha_aoa = theta - gamma
                q = 0.5 * RHO_AIR * speed2
                torque += q * area * cna * math.sin(alpha_aoa) * (cp - cm)
        omega += torque / inertia * dt
        theta += omega * dt

        # ---- translational dynamics (for drift/altitude display) ----
        drag = 0.5 * RHO_AIR * v_vert * abs(v_vert) * cd * area
        a_vert = (thr * math.cos(theta) - drag) / m - G
        if alt <= 0 and a_vert < 0:
            a_vert = 0.0  # still on the pad
        a_lat = thr * math.sin(theta) / m if burning else 0.0
        v_vert += a_vert * dt
        v_lat += a_lat * dt
        alt += v_vert * dt
        drift += v_lat * dt

        tilt_deg = math.degrees(theta)
        max_tilt = max(max_tilt, abs(tilt_deg))
        if abs(tilt_deg) > 1.0:
            settle_t = None
        elif settle_t is None:
            settle_t = t

        if step % rec_every == 0:
            out["t"].append(round(t, 4))
            out["tilt"].append(round(tilt_deg, 3))
            out["gimbal"].append(round(math.degrees(gimbal), 3))
            out["cmd"].append(round(math.degrees(cmd), 3))
            out["alt"].append(round(alt, 2))
            out["drift"].append(round(drift, 2))
        step += 1
        t += dt

    return {
        "series": out,
        "mass_props": {
            "mass_g": round(m0 * 1000, 1),
            "cm_cm": round(cm * 100, 2),
            "inertia_kgm2": round(i0, 6),
            "inertia_gcm2": round(i0 * 1e7, 0),
        },
        "metrics": {
            "max_tilt_deg": round(max_tilt, 2),
            "tilt_at_burnout_deg": round(out["tilt"][-1], 2) if out["tilt"] else None,
            "settle_s": round(settle_t, 2) if settle_t is not None else None,
            "saturation_pct": round(100.0 * sat_steps / max(ctrl_steps, 1), 1),
            "alt_burnout_m": round(out["alt"][-1], 1) if out["alt"] else None,
            "drift_burnout_m": round(out["drift"][-1], 1) if out["drift"] else None,
            "twr": round(thrust / (m0 * G), 2),
        },
    }


# ---------------------------------------------------------------------------
# Gain suggestion (pole placement on the rigid-body model)
# ---------------------------------------------------------------------------
def suggest_gains(p: dict) -> dict:
    mp_ = mass_properties(p)
    m, cm, inertia = mp_["mass_kg"], mp_["cm_m"], mp_["inertia_kgm2"]
    thrust = float(p["thrust_n"])
    burn = float(p["burn_s"])
    # Plant: I * theta'' = T * L * gimbal   =>  theta'' = K * gimbal
    K = thrust * cm / inertia  # (rad/s^2) per rad of gimbal
    if K <= 0:
        raise ValueError("Thrust, CM and inertia must be positive.")

    # If the rocket is aerodynamically unstable (CP above CM), the airflow
    # adds a destabilizing stiffness that grows with speed. Kp must supply
    # at least that much stiffness ON TOP of the wanted closed-loop wn^2,
    # sized at the worst case (burnout speed).
    ka = 0.0
    cp = float(p.get("cp_from_nozzle_cm", 0)) / 100.0
    if p.get("aero_enabled") and cp > cm:
        diameter = float(p.get("body_diameter_cm", 7.5)) / 100.0
        area = math.pi * (diameter / 2) ** 2
        cna = float(p.get("cna", 2.0))
        v_burnout = max((thrust / m - G) * burn, 0.0)  # rough, drag ignored
        q = 0.5 * RHO_AIR * v_burnout**2
        ka = q * area * cna * (cp - cm) / inertia      # rad/s^2 per rad

    wn = 2 * math.pi * float(p.get("target_bw_hz", 0.8))   # natural frequency
    zeta = float(p.get("target_zeta", 0.9))                # damping ratio
    kp = (wn * wn + ka) / K
    kd = 2 * zeta * wn / K
    ki = kp * wn / 10.0   # slow trim integrator for constant disturbances
    max_ang_acc = K * math.sin(math.radians(float(p["gimbal_limit_deg"])))
    note = (f"Poles placed at wn={wn:.1f} rad/s, zeta={zeta}. "
            f"Control authority: {math.degrees(max_ang_acc):.0f} deg/s^2 "
            f"at full gimbal.")
    if ka > 0:
        note += (f" Aero instability adds {ka:.0f} rad/s^2/rad at burnout "
                 f"speed — Kp raised to compensate.")
        # Above this tilt, the aero moment overpowers a fully-deflected
        # nozzle at burnout speed and the rocket is unrecoverable.
        ctrl_limit_deg = math.degrees(min(max_ang_acc / ka, math.pi / 2))
        if ctrl_limit_deg < 10:
            note += (f" WARNING: at burnout speed the TVC can only overcome "
                     f"the aero moment below ~{ctrl_limit_deg:.1f} deg of "
                     f"tilt — this design is marginal/uncontrollable. Add "
                     f"nose weight (CM up), reduce speed (smaller motor / "
                     f"more mass), or increase the gimbal limit.")
    return {
        "kp": round(kp, 4),
        "ki": round(ki, 4),
        "kd": round(kd, 4),
        "plant_gain": round(K, 2),
        "aero_stiffness": round(ka, 2),
        "max_ang_accel_dps2": round(math.degrees(max_ang_acc), 1),
        "note": note,
    }


# ---------------------------------------------------------------------------
# config.h writer
# ---------------------------------------------------------------------------
def write_config(values: dict) -> list[str]:
    """Replace #define values in config.h. Returns list of changed defines."""
    text = CONFIG_H.read_text()
    changed = []
    for macro, (val, is_float) in values.items():
        formatted = f"{val:.4f}f" if is_float else str(int(val))
        pattern = re.compile(rf"(#define\s+{macro}\s+)[-\w.+]+")
        new_text, n = pattern.subn(rf"\g<1>{formatted}", text, count=1)
        if n:
            text = new_text
            changed.append(f"{macro} = {formatted}")
    CONFIG_H.write_text(text)
    return changed


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template("index.html")


@app.post("/api/simulate")
def api_simulate():
    try:
        return jsonify(simulate(request.get_json(force=True)))
    except (ValueError, KeyError) as e:
        return jsonify({"error": str(e)}), 400


@app.post("/api/suggest")
def api_suggest():
    try:
        return jsonify(suggest_gains(request.get_json(force=True)))
    except (ValueError, KeyError) as e:
        return jsonify({"error": str(e)}), 400


@app.post("/api/apply")
def api_apply():
    p = request.get_json(force=True)
    try:
        values = {
            "PID_KP": (float(p["kp"]), True),
            "PID_KI": (float(p["ki"]), True),
            "PID_KD": (float(p["kd"]), True),
            "PID_I_LIMIT_DEG": (float(p["i_limit"]), True),
            "GIMBAL_LIMIT_DEG": (float(p["gimbal_limit_deg"]), True),
            "SERVO_GEAR_RATIO": (float(p["gear_ratio"]), True),
            "DTERM_LPF_HZ": (float(p["dterm_lpf_hz"]), True),
            "NOZZLE_RATE_LIMIT_DPS": (float(p["nozzle_rate_limit_dps"]), True),
            "TVC_ENABLE_DELAY_MS": (float(p["tvc_delay_ms"]), False),
            "MAX_BURN_MS": (float(p["burn_s"]) * 1000 + 500, False),
        }
        changed = write_config(values)
        return jsonify({"changed": changed, "path": str(CONFIG_H)})
    except (ValueError, KeyError) as e:
        return jsonify({"error": str(e)}), 400


@app.get("/api/ports")
def api_ports():
    from serial.tools import list_ports
    ports = [
        {"device": p.device, "desc": p.description}
        for p in list_ports.comports()
        if "Bluetooth" not in p.device
    ]
    return jsonify({"ports": ports, "arduino_cli": shutil.which("arduino-cli") is not None})


@app.post("/api/upload")
def api_upload():
    """Compile and (optionally) upload, streaming console output."""
    p = request.get_json(force=True)
    do_upload = bool(p.get("upload", True))
    port = p.get("port") or None

    def run(cmd):
        yield f"$ {' '.join(cmd)}\n"
        proc = subprocess.Popen(
            cmd, cwd=str(ROOT), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            yield line
        proc.wait()
        yield f"[exit code {proc.returncode}]\n"
        return proc.returncode

    def generate():
        if shutil.which("arduino-cli") is None:
            yield "ERROR: arduino-cli not found. Install it and the Teensy core (see README).\n"
            return
        compile_cmd = ["arduino-cli", "compile", "--fqbn", FQBN, str(SKETCH_DIR)]
        rc = yield from run(compile_cmd)
        if rc != 0:
            yield "\nCompile FAILED — not uploading.\n"
            return
        if not do_upload:
            yield "\nCompile OK.\n"
            return
        upload_cmd = ["arduino-cli", "upload", "--fqbn", FQBN]
        if port:
            upload_cmd += ["-p", port]
        upload_cmd.append(str(SKETCH_DIR))
        rc = yield from run(upload_cmd)
        yield ("\nUpload OK — the board should reboot with the new firmware.\n"
               if rc == 0 else
               "\nUpload FAILED. Is the Teensy plugged in? Press its button "
               "once and retry.\n")

    return Response(stream_with_context(generate()), mimetype="text/plain")


@app.post("/api/live")
def api_live():
    """Push gains to a running board over USB serial (set ... / save)."""
    import serial

    p = request.get_json(force=True)
    port = p.get("port")
    if not port:
        return jsonify({"error": "No serial port selected."}), 400
    cmds = [
        f"set kp {float(p['kp']):.4f}",
        f"set ki {float(p['ki']):.4f}",
        f"set kd {float(p['kd']):.4f}",
        f"set gear {float(p['gear_ratio']):.3f}",
    ]
    if p.get("persist"):
        cmds.append("save")
    transcript = []
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            time.sleep(0.3)
            ser.reset_input_buffer()
            for c in cmds:
                ser.write((c + "\n").encode())
                ser.flush()
                time.sleep(0.15)
                resp = ser.read_all().decode(errors="replace").strip()
                transcript.append(f"> {c}\n{resp}")
    except serial.SerialException as e:
        return jsonify({"error": f"Serial error: {e}"}), 500
    return jsonify({"transcript": "\n".join(transcript)})


if __name__ == "__main__":
    print(f"Firmware sketch: {SKETCH_DIR}")
    print("TVC Tuner running — open http://127.0.0.1:5055")
    app.run(host="127.0.0.1", port=5055, debug=False)
