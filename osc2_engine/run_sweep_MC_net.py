#!/usr/bin/env python3
"""
Activity 3 — MC sweep over network-side V2X imperfections.

Thin wrapper around run_sweep_MC.py that adds:

  1. A V2X network middlebox (v2x_net_middlebox.py) which intercepts the
     bridge's CAM output and applies per-message imperfections:
        - drop_prob   : Bernoulli packet loss
        - delay_ms    : extra wall-clock latency
        - pos_noise_m : Gaussian noise on ped (x, y)

  2. An env-var hook (V2X_BRIDGE_EXTRA_PARAMS) read by run_sweep.py to
     remap the bridge's cam_topic to /v2x/cam_received_raw, so the middlebox
     can sit between bridge and AEB without touching either of them.

  3. Sweep mode: pass --net-drop-prob-sweep / --net-delay-ms-sweep /
     --net-pos-noise-m-sweep with space- or comma-separated values to run
     a Cartesian grid in a single invocation. The middlebox is restarted
     per cell so per-message seeds stay reproducible.

When this script is NOT used, run_sweep_MC.py and run_sweep.py behave
bit-for-bit identically to before. All Activity-3 logic is here.

Examples:
    # single cell
    python3 osc2_engine/run_sweep_MC_net.py \\
        --speeds 40 --trigger-distances 20.0 \\
        --monte-carlo 15 --seed 42 --v2x \\
        --net-drop-prob 0.3 --net-delay-ms 80 \\
        --scenario osc2_engine/scenarios/vru_ped_occluded_parked_bus_realistic.osc

    # univariate drop sweep at 40 km/h
    python3 osc2_engine/run_sweep_MC_net.py \\
        --speeds 40 --trigger-distances 20.0 \\
        --monte-carlo 15 --seed 42 --v2x \\
        --net-drop-prob-sweep "0 0.1 0.2 0.3 0.5 0.7 0.9" \\
        --scenario osc2_engine/scenarios/vru_ped_occluded_parked_bus_realistic.osc

    # univariate delay sweep at 40 km/h
    python3 osc2_engine/run_sweep_MC_net.py \\
        --speeds 40 --trigger-distances 20.0 \\
        --monte-carlo 15 --seed 42 --v2x \\
        --net-delay-ms-sweep "0 50 100 200 400 800 1500" \\
        --scenario osc2_engine/scenarios/vru_ped_occluded_parked_bus_realistic.osc
"""

import argparse
import os
import signal
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR   = os.path.dirname(SCRIPT_DIR)

MIDDLEBOX_PATH = os.path.join(SCRIPT_DIR, "v2x_net_middlebox.py")


def split_net_args(argv):
    """Strip Activity-3-specific flags out of argv; return (net_opts, rest)."""
    p = argparse.ArgumentParser(add_help=False)
    # Single-value flags
    p.add_argument("--net-drop-prob",     type=float, default=None)
    p.add_argument("--net-delay-ms",      type=float, default=None)
    p.add_argument("--net-pos-noise-m",   type=float, default=None)
    p.add_argument("--net-burst-start-s", type=float, default=None,
                   help="Start of correlated-loss window (s).")
    p.add_argument("--net-burst-dur-s",   type=float, default=None,
                   help="Duration of correlated-loss window (s). 0 = off.")
    p.add_argument("--net-cam-rate-hz",   type=float, default=None,
                   help="Cap CAM rate (Hz). 0 or >=10 = pass-through.")
    p.add_argument("--net-seed",          type=int,   default=42)
    p.add_argument("--net-label",         type=str,   default=None,
                   help="Subdir under output/sweep_MC_v2x_net/ "
                        "(default auto-derived; ignored in sweep mode).")
    # Sweep flags (space- or comma-separated lists)
    p.add_argument("--net-drop-prob-sweep",   type=str, default=None,
                   help="Sweep drop_prob over these values "
                        "(e.g. '0 0.1 0.2 0.3 0.5 0.7 0.9').")
    p.add_argument("--net-delay-ms-sweep",    type=str, default=None,
                   help="Sweep delay_ms over these values "
                        "(e.g. '0 50 100 200 400 800 1500').")
    p.add_argument("--net-pos-noise-m-sweep", type=str, default=None,
                   help="Sweep pos_noise_m over these values "
                        "(e.g. '0 0.25 0.5 1.0 2.0').")
    net_opts, rest = p.parse_known_args(argv)
    return net_opts, rest


def parse_sweep_list(s):
    """Parse '0 0.1 0.2' or '0,0.1,0.2' into [0.0, 0.1, 0.2]. None passthrough."""
    if s is None:
        return None
    s = s.replace(",", " ").strip()
    if not s:
        return None
    return [float(tok) for tok in s.split()]


def _prompt_float(prompt, default):
    raw = input(f"  {prompt} [{default}]: ").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        print(f"  (invalid number, using {default})")
        return default


# Menu of available perturbations. Each entry says how to label it, which
# net_opts fields it owns, and which params to ask for when enabled.
#
# Key insight: every field defaults to 0.0 (= disabled in the middlebox).
# The user only ever sets the fields belonging to the perturbations they
# pick from the menu — everything else stays at 0.
PERTURBATION_MENU = [
    {
        "key":   "drop",
        "title": "Bernoulli packet drop",
        "blurb": "independent per-CAM loss",
        "fields": ["net_drop_prob"],
        "ask":    lambda o: setattr(o, "net_drop_prob",
                    _prompt_float("Packet drop probability (0.0 .. 1.0)", 0.5)),
    },
    {
        "key":   "delay",
        "title": "Constant link delay",
        "blurb": "fixed extra latency per CAM",
        "fields": ["net_delay_ms"],
        "ask":    lambda o: setattr(o, "net_delay_ms",
                    _prompt_float("Extra latency per CAM (ms)", 500.0)),
    },
    {
        "key":   "noise",
        "title": "Gaussian position noise",
        "blurb": "sensor / localization error on RSU-perceived ped",
        "fields": ["net_pos_noise_m"],
        "ask":    lambda o: setattr(o, "net_pos_noise_m",
                    _prompt_float("Ped position noise σ (m)", 1.0)),
    },
    {
        "key":   "burst",
        "title": "Burst (correlated) packet loss",
        "blurb": "deterministic blackout window — channel goes dead",
        "fields": ["net_burst_start_s", "net_burst_dur_s"],
        "ask":    lambda o: (
            setattr(o, "net_burst_start_s",
                    _prompt_float("Blackout start (s, from middlebox boot)", 3.0)),
            setattr(o, "net_burst_dur_s",
                    _prompt_float("Blackout duration (s)", 3.0)),
        ),
    },
    {
        "key":   "rate",
        "title": "Reduced CAM rate",
        "blurb": "downsample 10 Hz output to a slower channel rate",
        "fields": ["net_cam_rate_hz"],
        "ask":    lambda o: setattr(o, "net_cam_rate_hz",
                    _prompt_float("Target CAM rate (Hz, < 10)", 1.0)),
    },
]


def _parse_menu_selection(raw, n_items):
    """'1 3 4' or '1,3,4' -> [0, 2, 3]. Returns [] if blank or invalid."""
    raw = raw.replace(",", " ").strip()
    if not raw:
        return []
    out = []
    for tok in raw.split():
        try:
            i = int(tok) - 1
        except ValueError:
            continue
        if 0 <= i < n_items and i not in out:
            out.append(i)
    return out


def _zero_unset_fields(net_opts):
    """Replace any leftover None on the perturbation knobs with 0.0."""
    for field in ("net_drop_prob", "net_delay_ms", "net_pos_noise_m",
                  "net_burst_start_s", "net_burst_dur_s", "net_cam_rate_hz"):
        if getattr(net_opts, field, None) is None:
            setattr(net_opts, field, 0.0)
    return net_opts


def prompt_net_perturbations(net_opts):
    """If no net-* CLI flag is given, ask interactively. Skipped in sweep mode."""
    sweep_mode = (net_opts.net_drop_prob_sweep is not None
                  or net_opts.net_delay_ms_sweep is not None
                  or net_opts.net_pos_noise_m_sweep is not None)
    if sweep_mode:
        return _zero_unset_fields(net_opts)

    # Any single perturbation knob set on CLI = skip prompt entirely.
    cli_set = any(getattr(net_opts, f) is not None for f in (
        "net_drop_prob", "net_delay_ms", "net_pos_noise_m",
        "net_burst_start_s", "net_burst_dur_s", "net_cam_rate_hz",
        "net_label",
    ))
    if cli_set:
        return _zero_unset_fields(net_opts)

    print()
    print("=" * 70)
    print("  V2X NETWORK PERTURBATIONS  (Activity 3)")
    print("=" * 70)
    print("  Pick which perturbations to apply (you can combine any of them).")
    print("  Press Enter on a blank line for a CLEAN V2X control run.")
    print()
    for i, item in enumerate(PERTURBATION_MENU, start=1):
        print(f"    [{i}] {item['title']:<32s} — {item['blurb']}")
    print()

    raw = input("  Enable which? (e.g. '1 3 4', or Enter for none): ").strip()
    enabled = _parse_menu_selection(raw, len(PERTURBATION_MENU))

    if not enabled:
        print("  → No perturbations enabled (clean V2X run).")
        net_opts = _zero_unset_fields(net_opts)
    else:
        names = [PERTURBATION_MENU[i]["key"] for i in enabled]
        print(f"  → Enabled: {', '.join(names)}")
        print()
        for i in enabled:
            item = PERTURBATION_MENU[i]
            print(f"  ─ {item['title']} ─")
            item["ask"](net_opts)
        net_opts = _zero_unset_fields(net_opts)

    print()
    label_in = input("  Run label (Enter = auto from values): ").strip()
    net_opts.net_label = label_in or None
    print()
    return net_opts


def derive_label(net):
    if net.net_label:
        return net.net_label
    parts = []
    if net.net_drop_prob > 0:    parts.append(f"drop{int(round(net.net_drop_prob*100)):02d}")
    if net.net_delay_ms > 0:     parts.append(f"d{int(round(net.net_delay_ms)):04d}ms")
    if net.net_pos_noise_m > 0:  parts.append(f"pos{net.net_pos_noise_m:.2f}m".replace(".", "p"))
    if net.net_burst_dur_s > 0:
        parts.append(f"burst{int(round(net.net_burst_start_s)):02d}+"
                     f"{int(round(net.net_burst_dur_s)):02d}s")
    if 0 < net.net_cam_rate_hz < 10:
        parts.append(f"rate{net.net_cam_rate_hz:.1f}hz".replace(".", "p"))
    return "_".join(parts) if parts else "clean"


def start_middlebox(net, log_dir):
    """Spawn the middlebox node and return the Popen handle."""
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, "v2x_net_middlebox.log")
    log_fh   = open(log_path, "w")
    cmd = [
        sys.executable, MIDDLEBOX_PATH,
        "--in-topic",       "/v2x/cam_received_raw",
        "--out-topic",      "/v2x/cam_received",
        "--drop-prob",      str(net.net_drop_prob),
        "--delay-ms",       str(net.net_delay_ms),
        "--pos-noise-m",    str(net.net_pos_noise_m),
        "--burst-start-s",  str(net.net_burst_start_s),
        "--burst-dur-s",    str(net.net_burst_dur_s),
        "--cam-rate-hz",    str(net.net_cam_rate_hz),
        "--seed",           str(net.net_seed),
    ]
    print(f"[net-mc] launching middlebox: {' '.join(cmd)}")
    print(f"[net-mc]   log → {log_path}")
    proc = subprocess.Popen(
        cmd, stdout=log_fh, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
    )
    time.sleep(2.0)
    return proc, log_fh


def kill_middlebox(proc, log_fh):
    if proc is None:
        return
    try:
        print("[net-mc] stopping middlebox")
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    finally:
        try:
            log_fh.close()
        except Exception:
            pass


def build_cell_grid(net_opts):
    """Return (drops, delays, noises, cells). Sweep flag → list; else scalar."""
    drops  = parse_sweep_list(net_opts.net_drop_prob_sweep)   or [net_opts.net_drop_prob]
    delays = parse_sweep_list(net_opts.net_delay_ms_sweep)    or [net_opts.net_delay_ms]
    noises = parse_sweep_list(net_opts.net_pos_noise_m_sweep) or [net_opts.net_pos_noise_m]
    cells  = [(d, ms, n) for d in drops for ms in delays for n in noises]
    return drops, delays, noises, cells


def strip_output_dir(argv):
    """Drop any '--output-dir VALUE' pair so we can re-append per cell."""
    out, skip = [], False
    for a in argv:
        if skip:
            skip = False
            continue
        if a == "--output-dir":
            skip = True
            continue
        out.append(a)
    return out


def main():
    # 1. Pull out Activity-3 flags so they don't reach run_sweep_MC's parser.
    net_opts, rest_argv = split_net_args(sys.argv[1:])
    net_opts = prompt_net_perturbations(net_opts)

    drops, delays, noises, cells = build_cell_grid(net_opts)
    sweep_mode = len(cells) > 1

    # Force --v2x if user forgot — Activity 3 only makes sense on V2X.
    if "--v2x" not in rest_argv and "--v2x-fusion" not in rest_argv:
        print("[net-mc] note: --v2x not supplied — forcing --v2x for Activity 3")
        rest_argv.append("--v2x")

    if sweep_mode:
        print()
        print("=" * 70)
        print(f"  V2X PERTURBATION SWEEP — {len(cells)} cells")
        print("=" * 70)
        print(f"  drops  : {drops}")
        print(f"  delays : {delays}")
        print(f"  noises : {noises}")
        print(f"  reps/cell = --monte-carlo arg")
        print(f"  parent dir = {os.path.join(ROOT_DIR, 'output', 'sweep_MC_v2x_net')}")
        print()

    # 2. Tell run_sweep.py to remap the bridge cam_topic (env-var hook).
    os.environ["V2X_BRIDGE_EXTRA_PARAMS"] = "-p cam_topic:=/v2x/cam_received_raw"
    print(f"[net-mc] V2X_BRIDGE_EXTRA_PARAMS = {os.environ['V2X_BRIDGE_EXTRA_PARAMS']}")

    # Capture original parse_args once; restore in finally.
    import run_sweep_MC
    _orig_parse_args = run_sweep_MC.parse_args

    try:
        for cell_idx, (drop, delay, noise) in enumerate(cells, start=1):
            net_opts.net_drop_prob   = drop
            net_opts.net_delay_ms    = delay
            net_opts.net_pos_noise_m = noise
            if sweep_mode:
                # In sweep mode the label is auto-derived per cell; ignore any
                # static --net-label given on CLI so output dirs don't collide.
                net_opts.net_label = None
            label  = derive_label(net_opts)
            outdir = os.path.join(ROOT_DIR, "output", "sweep_MC_v2x_net", label)
            os.makedirs(outdir, exist_ok=True)

            extras = []
            if net_opts.net_burst_dur_s > 0:
                extras.append(f"burst=[{net_opts.net_burst_start_s}s,"
                              f" +{net_opts.net_burst_dur_s}s]")
            if 0 < net_opts.net_cam_rate_hz < 10:
                extras.append(f"cam_rate={net_opts.net_cam_rate_hz}Hz")
            extras_str = ("  " + "  ".join(extras)) if extras else ""

            print()
            print("=" * 70)
            print(f"  CELL {cell_idx}/{len(cells)}  label={label}")
            print(f"  drop_prob={drop}  delay_ms={delay}  pos_noise_m={noise}"
                  f"{extras_str}")
            print(f"  output → {outdir}")
            print("=" * 70)

            # Per-cell argv: replace any prior --output-dir, then inject ours.
            cell_argv = strip_output_dir(rest_argv) + ["--output-dir", outdir]
            sys.argv = [sys.argv[0]] + cell_argv

            # parse_args() (and its interactive branch) re-derives output_dir
            # from defaults and ignores our --output-dir injection. Monkey-
            # patch it so the per-cell CSV lands in our Activity-3 dir
            # alongside the middlebox log + per-run dirs.
            def _make_patched(outdir_, orig_):
                def _patched():
                    args = orig_()
                    args.output_dir = outdir_
                    os.makedirs(args.output_dir, exist_ok=True)
                    return args
                return _patched
            run_sweep_MC.parse_args = _make_patched(outdir, _orig_parse_args)

            mb_proc, mb_log = start_middlebox(net_opts, outdir)
            try:
                run_sweep_MC.main()
            except SystemExit as e:
                # run_sweep_MC may call sys.exit() at end of a cell; that's OK.
                code = e.code if e.code is not None else 0
                print(f"[net-mc] cell {cell_idx}: run_sweep_MC.main() exited "
                      f"with code {code}")
            except Exception as ex:
                print(f"[net-mc] cell {cell_idx}: ERROR {type(ex).__name__}: {ex}")
                print(f"[net-mc] continuing to next cell")
            finally:
                kill_middlebox(mb_proc, mb_log)
    finally:
        run_sweep_MC.parse_args = _orig_parse_args
        os.environ.pop("V2X_BRIDGE_EXTRA_PARAMS", None)
        if sweep_mode:
            print()
            print("=" * 70)
            print(f"  SWEEP COMPLETE — {len(cells)} cells")
            print(f"  results in: {os.path.join(ROOT_DIR, 'output', 'sweep_MC_v2x_net')}")
            print("=" * 70)


if __name__ == "__main__":
    main()
