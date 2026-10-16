#!/usr/bin/env python3
"""PID tuning and step-response checks against the software-in-the-loop sim.

Every number this tool reads or writes is SIMULATED: it comes from fc_sil, the
flight firmware closed around an assumed quadrotor model on the host
(firmware/sim/sil). Nothing here was measured on hardware.

Subcommands
  run     run fc_sil with the firmware defaults (or given gains), write CSVs and
          metrics.json to an output directory
  tune    coordinate-descent search over the rate and angle gains, starting
          from the firmware defaults, against the same limits CI gates on
  check   fail unless every scenario in a metrics.json is within its limits;
          with --reference, also fail if the metrics drift from a committed copy
  plot    draw the step responses in a results directory to PNG (needs
          matplotlib, see tools/requirements.txt)

Standard library only, except for plot.
"""

import argparse
import json
import math
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SIL = os.path.join(REPO, "firmware", "build", "fc_sil")
DEFAULT_OUT = os.path.join(REPO, "test-results", "sil")


def rel(path):
    """Repository-relative POSIX path for anything printed or recorded."""
    return os.path.relpath(path, REPO).replace(os.sep, "/")


# --------------------------------------------------------------------------
# Running the simulator


def gains_to_args(g):
    args = []
    if g is None:
        return args
    for key, flag in (("rate_roll", "--rate-roll"), ("rate_pitch", "--rate-pitch"),
                      ("rate_yaw", "--rate-yaw")):
        if key in g:
            args += [flag, ",".join("%.6g" % v for v in g[key])]
    if "angle_p" in g:
        args += ["--angle-p", "%.6g" % g["angle_p"]]
    return args


# The tuner scores every candidate on the nominal plant and on a deliberately
# slower, heavier one, and keeps the worse score. CI checks the same variant,
# so the committed gains have to carry some margin rather than fit one model.
PERTURBED = ["--motor-tau", "0.045", "--inertia-scale", "1.3"]


def run_sil(sil, gains=None, out_dir=None, extra=None):
    cmd = [sil, "--json"] + gains_to_args(gains) + (extra or [])
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        cmd += ["--out", out_dir]
    proc = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return json.loads(proc.stdout)


# --------------------------------------------------------------------------
# Scoring


def scenario_cost(s):
    lim = s["limits"]
    if not s["armed"] or not s["bounded"] or s["settling_time_s"] is None:
        sse = s["steady_state_error_pct"] or 100.0
        return 20.0 + sse / lim["max_steady_state_error_pct"]
    settle = s["settling_time_s"] / lim["max_settling_time_s"]
    over = s["overshoot_pct"] / lim["max_overshoot_pct"]
    sse = s["steady_state_error_pct"] / lim["max_steady_state_error_pct"]
    cost = settle + 0.5 * over + 0.5 * sse
    # Squared hinge on every limit, so a pass always beats a near miss.
    for ratio in (settle, over, sse):
        if ratio > 1.0:
            cost += 10.0 * (ratio - 1.0) ** 2 + 2.0
    return cost


def total_cost(metrics):
    return sum(scenario_cost(s) for s in metrics["scenarios"])


def activity(metrics):
    vals = [s["hover_motor_activity"] for s in metrics["scenarios"]]
    return sum(vals) / len(vals)


def robust_cost(nominal, perturbed, reference_activity):
    """Worse of the two plants, plus a price on motor command activity in
    hover: sensor noise pushed through high gains is heat and wear on a real
    aircraft, and the model cannot see either."""
    ratio = activity(nominal) / reference_activity
    cost = max(total_cost(nominal), total_cost(perturbed)) + 0.5 * ratio
    if ratio > 3.0:
        cost += 10.0 * (ratio - 3.0) ** 2
    return cost


def evaluate(sil, gains):
    return run_sil(sil, gains), run_sil(sil, gains, extra=PERTURBED)


def all_pass(metrics):
    return all(s["pass"] for s in metrics["scenarios"])


# --------------------------------------------------------------------------
# Search
#
# Roll and pitch share gains: the assumed airframe is symmetric, and tuning
# them apart would only fit noise in the model.

PARAMS = [
    ("rp_kp", lambda g: g["rate_roll"][0]),
    ("rp_ki", lambda g: g["rate_roll"][1]),
    ("rp_kd", lambda g: g["rate_roll"][2]),
    ("yaw_kp", lambda g: g["rate_yaw"][0]),
    ("yaw_ki", lambda g: g["rate_yaw"][1]),
    ("angle_p", lambda g: g["angle_p"]),
]


def vector_from_gains(g):
    return [get(g) for _, get in PARAMS]


def gains_from_vector(v):
    rp = [v[0], v[1], v[2]]
    return {
        "rate_roll": rp,
        "rate_pitch": list(rp),
        "rate_yaw": [v[3], v[4], 0.0],
        "angle_p": v[5],
    }


def tune(sil, max_evals, log, start_gains=None):
    baseline, baseline_pert = evaluate(sil, start_gains)
    ref_activity = activity(baseline)
    start = vector_from_gains(baseline["gains"])
    best_v = start
    best_m = baseline
    best_c = robust_cost(baseline, baseline_pert, ref_activity)
    evals = 1
    both = all_pass(baseline) and all_pass(baseline_pert)
    history = [{"eval": evals, "gains": gains_from_vector(best_v), "cost": round(best_c, 4),
                "pass": both}]
    log("start  cost %.3f  pass=%s  %s" % (best_c, both, fmt(best_v)))

    step = 1.4
    while evals < max_evals and step > 1.04:
        improved = False
        for i, (name, _) in enumerate(PARAMS):
            for factor in (step, 1.0 / step):
                if evals >= max_evals:
                    break
                cand = list(best_v)
                cand[i] = cand[i] * factor
                m, mp = evaluate(sil, gains_from_vector(cand))
                c = robust_cost(m, mp, ref_activity)
                evals += 1
                both = all_pass(m) and all_pass(mp)
                history.append({"eval": evals, "gains": gains_from_vector(cand),
                                "cost": round(c, 4), "pass": both})
                if c < best_c - 1e-6:
                    best_v, best_m, best_c = cand, m, c
                    improved = True
                    log("eval %3d  cost %.3f  pass=%s  %s=%.4g" % (evals, c, both, name,
                                                                  cand[i]))
                    break
        if not improved:
            step = math.sqrt(step)
    return baseline, best_m, history


def fmt(v):
    return " ".join("%s=%.4g" % (name, x) for (name, _), x in zip(PARAMS, v))


# --------------------------------------------------------------------------
# Commands


def cmd_run(args):
    m = run_sil(args.sil, load_gains(args.gains), args.out)
    print_table(m)
    print("wrote %s" % rel(args.out))
    return 0 if all_pass(m) else 1


def cmd_tune(args):
    baseline, best, history = tune(args.sil, args.max_evals, print, load_gains(args.start))
    print()
    print("starting point (%s):" % (rel(os.path.abspath(args.start)) if args.start
                                    else "firmware defaults"))
    print_table(baseline)
    print("best found (nominal plant):")
    print_table(best)
    print("best found (perturbed plant: %s):" % " ".join(PERTURBED))
    print_table(run_sil(args.sil, best["gains"], extra=PERTURBED))
    g = best["gains"]
    print("FlightControllerConfig values (simulated tuning, not flight validated):")
    print("    PidGains rate_roll{%.4gf, %.4gf, %.4gf};" % tuple(g["rate_roll"]))
    print("    PidGains rate_pitch{%.4gf, %.4gf, %.4gf};" % tuple(g["rate_pitch"]))
    print("    PidGains rate_yaw{%.4gf, %.4gf, %.4gf};" % tuple(g["rate_yaw"]))
    print("    float angle_p{%.4gf};" % g["angle_p"])
    if args.record:
        record = {
            "simulated": True,
            "note": "Gain search against fc_sil and an assumed plant. Not flight data.",
            "start": baseline["gains"],
            "best": best["gains"],
            "perturbed_plant_args": PERTURBED,
            "evaluations": history,
        }
        os.makedirs(os.path.dirname(args.record), exist_ok=True)
        with open(args.record, "w", newline="\n") as f:
            json.dump(record, f, indent=1)
            f.write("\n")
        print("wrote %s" % rel(args.record))
    return 0


def cmd_check(args):
    with open(args.metrics) as f:
        m = json.load(f)
    ok = m.get("simulated") is True
    if not ok:
        print("metrics are not labelled simulated")
    print_table(m)
    if not all_pass(m):
        ok = False
        print("FAIL: at least one scenario is outside its limits")
    if args.reference:
        with open(args.reference) as f:
            ref = json.load(f)
        drift = compare(ref, m, args.tolerance)
        for line in drift:
            print("drift: " + line)
        if drift:
            ok = False
            print("FAIL: regenerated metrics differ from %s; rerun the sim and commit"
                  % rel(os.path.abspath(args.reference)))
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def compare(ref, new, tol):
    """Numeric fields must agree within tol (absolute or relative, whichever is
    looser). Floating point results are not bit reproducible across compilers."""
    out = []
    by_name = {s["name"]: s for s in new["scenarios"]}
    if ref["gains"] != new["gains"]:
        out.append("gains %s -> %s" % (ref["gains"], new["gains"]))
    for s in ref["scenarios"]:
        n = by_name.get(s["name"])
        if n is None:
            out.append("%s missing" % s["name"])
            continue
        for key, a in s.items():
            b = n.get(key)
            if isinstance(a, bool) or not isinstance(a, (int, float)):
                if a != b and key != "limits":
                    out.append("%s.%s %r -> %r" % (s["name"], key, a, b))
                continue
            if b is None or abs(a - b) > max(tol, tol * abs(a)):
                out.append("%s.%s %r -> %r" % (s["name"], key, a, b))
    return out


def cmd_plot(args):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("plot needs matplotlib: pip install -r tools/requirements.txt")
        return 2
    with open(os.path.join(args.dir, "metrics.json")) as f:
        m = json.load(f)
    for s in m["scenarios"]:
        path = os.path.join(args.dir, s["name"] + ".csv")
        t, sp, resp, est = [], [], [], []
        units = "deg"
        with open(path) as f:
            for line in f:
                if line.startswith("#"):
                    if "units=deg/s" in line:
                        units = "deg/s"
                    continue
                if line.startswith("t_s"):
                    continue
                row = line.split(",")
                t.append(float(row[0]))
                sp.append(float(row[1]))
                resp.append(float(row[2]))
                est.append(float(row[3]))
        fig, ax = plt.subplots(figsize=(7, 3.6))
        ax.plot(t, sp, color="#555555", linestyle="--", linewidth=1.2, label="setpoint")
        ax.plot(t, resp, color="#1f6feb", linewidth=1.6, label="true (plant)")
        label = "firmware estimate" if s["judged_on"] == "estimate" else "gyro reading"
        ax.plot(t, est, color="#d97706", linewidth=1.0, alpha=0.9, label=label)
        verdict = "pass" if s["pass"] else "FAIL"
        ax.set_title("SIMULATED  %s  (%s, overshoot %.1f %%, settling %s)" % (
            s["name"], verdict, s["overshoot_pct"],
            "%.3f s" % s["settling_time_s"] if s["settling_time_s"] is not None else "none"),
            fontsize=10)
        ax.set_xlabel("time since release into hover (s)")
        ax.set_ylabel(units)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")
        fig.text(0.01, 0.01, "Software-in-the-loop, assumed plant. Not hardware data.",
                 fontsize=7, color="#666666")
        fig.tight_layout(rect=(0, 0.03, 1, 1))
        png = os.path.join(args.dir, s["name"] + ".png")
        fig.savefig(png, dpi=110, metadata={"Software": None})
        plt.close(fig)
        print("wrote %s" % rel(png))
    return 0


def print_table(m):
    print("  %-18s %-6s %10s %10s %10s %12s" % ("scenario", "result", "overshoot", "settling",
                                               "ss error", "est. error"))
    for s in m["scenarios"]:
        settle = "%.3f s" % s["settling_time_s"] if s["settling_time_s"] is not None else "never"
        print("  %-18s %-6s %8.2f %% %10s %8.2f %% %10.2f" % (
            s["name"], "pass" if s["pass"] else "FAIL", s["overshoot_pct"], settle,
            s["steady_state_error_pct"], s["max_estimator_error"]))
    print("  (simulated)")


def load_gains(path):
    if not path:
        return None
    with open(path) as f:
        return json.load(f)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--sil", default=DEFAULT_SIL, help="path to the fc_sil binary")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="run the standard scenarios")
    r.add_argument("--out", default=DEFAULT_OUT)
    r.add_argument("--gains", help="JSON file with gains to use instead of the defaults")
    r.set_defaults(func=cmd_run)

    t = sub.add_parser("tune", help="search for gains")
    t.add_argument("--max-evals", type=int, default=120)
    t.add_argument("--record", help="write the search history to this JSON file")
    t.add_argument("--start", help="JSON file of gains to start from instead of the defaults")
    t.set_defaults(func=cmd_tune)

    c = sub.add_parser("check", help="gate on a metrics.json")
    c.add_argument("metrics")
    c.add_argument("--reference", help="committed metrics.json to compare against")
    c.add_argument("--tolerance", type=float, default=1e-3)
    c.set_defaults(func=cmd_check)

    pl = sub.add_parser("plot", help="plot a results directory")
    pl.add_argument("--dir", default=DEFAULT_OUT)
    pl.set_defaults(func=cmd_plot)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
