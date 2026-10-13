#!/usr/bin/env python3
"""LoamAir client: fetch IMU telemetry from the device over Wi-Fi, log it to
disk (raw binary + decoded CSV) and render a waveform for observation.

The plot is written as an interactive, self-contained HTML file (no packages
required, opens in any browser). If matplotlib is available a PNG is produced
instead, unless --html is given.

Online (device softAP up, host default 192.168.4.1):
    python loamair_client.py --duration 20
    python loamair_client.py --host 192.168.4.1 --out logs/jump1

Offline (re-plot a previously captured raw log):
    python loamair_client.py --offline logs/jump1.bin
    python loamair_client.py --offline logs/jump1.bin --html
"""

import argparse
import json
import os
import struct
import sys
import time
import urllib.request
from datetime import datetime, timezone

RECORD_FORMAT = "<I6h"          # uint32 ts_offset_us + 6x int16
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)  # 16
ACCEL_LSB_PER_G = 2048.0        # +/-16 g
GYRO_LSB_PER_DPS = 16.4         # +/-2000 dps
CSV_HEADER = "ts_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n"

LABELS = ["ax", "ay", "az", "gx", "gy", "gz"]
UNITS = ["g", "g", "g", "dps", "dps", "dps"]
SCALES = [ACCEL_LSB_PER_G] * 3 + [GYRO_LSB_PER_DPS] * 3
MAX_EMBED_SAMPLES = 200_000      # decimate above this to keep the HTML responsive


def fetch_status(host, port, timeout=5.0):
    url = f"http://{host}:{port}/status"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001 - report and continue streaming
        print(f"warning: could not read /status ({exc})", file=sys.stderr)
        return None


def decode_into(fc, raw):
    """Decode whole 16-byte records from `raw` into CSV rows.

    Returns (rows_written, leftover_bytes).
    """
    count = len(raw) // RECORD_SIZE
    rows = 0
    for i in range(count):
        ts_us, ax, ay, az, gx, gy, gz = struct.unpack_from(RECORD_FORMAT, raw, i * RECORD_SIZE)
        fc.write(
            f"{ts_us / 1e6:.6f},"
            f"{ax / ACCEL_LSB_PER_G:.5f},{ay / ACCEL_LSB_PER_G:.5f},{az / ACCEL_LSB_PER_G:.5f},"
            f"{gx / GYRO_LSB_PER_DPS:.3f},{gy / GYRO_LSB_PER_DPS:.3f},{gz / GYRO_LSB_PER_DPS:.3f}\n"
        )
        rows += 1
    return rows, raw[count * RECORD_SIZE:]


def stream(host, port, out_prefix, duration):
    bin_path = out_prefix + ".bin"
    csv_path = out_prefix + ".csv"

    status = fetch_status(host, port)
    if status is not None:
        print(
            "device: {count}/{capacity} records buffered, {sample_rate_hz} Hz".format(**status)
        )

    os.makedirs(os.path.dirname(os.path.abspath(bin_path)), exist_ok=True)

    url = f"http://{host}:{port}/data"
    req = urllib.request.Request(url, headers={"User-Agent": "loamair-client"})

    total = 0
    leftover = b""
    started = time.time()

    print(f"streaming {url} -> {bin_path} (+ {csv_path})")
    if duration:
        print(f"duration: {duration:.0f}s (Ctrl+C to stop early)")
    else:
        print("streaming until Ctrl+C...")

    with open(bin_path, "wb") as fb, open(csv_path, "w", newline="") as fc:
        fc.write(CSV_HEADER)
        try:
            with urllib.request.urlopen(req, timeout=15) as resp:
                while True:
                    if duration and (time.time() - started) >= duration:
                        break
                    chunk = resp.read(4096)
                    if not chunk:
                        break
                    fb.write(chunk)
                    rows, leftover = decode_into(fc, leftover + chunk)
                    total += rows
        except KeyboardInterrupt:
            print("\nstopped by user")

    if leftover:
        print(f"warning: {len(leftover)} trailing bytes (partial record)", file=sys.stderr)

    print(f"captured {total} records ({total / 1e3:.1f} ksamples)")
    return bin_path, csv_path, total


def read_records(bin_path):
    """Return (ts_seconds, [6 series]) with time zeroed to the first sample."""
    with open(bin_path, "rb") as f:
        data = f.read()
    n = len(data) // RECORD_SIZE
    if n == 0:
        return [], [[] for _ in range(6)]

    ts = [0.0] * n
    series = [[0.0] * n for _ in range(6)]
    t0 = None
    for i, values in enumerate(struct.iter_unpack(RECORD_FORMAT, data[: n * RECORD_SIZE])):
        if t0 is None:
            t0 = values[0]
        ts[i] = (values[0] - t0) / 1e6
        for axis in range(6):
            series[axis][i] = values[axis + 1] / SCALES[axis]
    return ts, series


def _median_dt(ts):
    if len(ts) < 2:
        return 0.01
    diffs = sorted(ts[i + 1] - ts[i] for i in range(len(ts) - 1))
    med = diffs[len(diffs) // 2]
    return med if med > 0 else 0.01


def render_matplotlib(ts, series, png_path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001 - any import failure falls back to HTML
        print(f"matplotlib unavailable ({exc}); writing interactive HTML instead")
        return None

    fig, plots = plt.subplots(6, 1, sharex=True, figsize=(11, 9))
    for ax, data, label, unit in zip(plots, series, LABELS, UNITS):
        ax.plot(ts, data, linewidth=0.8)
        ax.set_ylabel(f"{label} ({unit})")
        ax.grid(True, alpha=0.3)
    plots[-1].set_xlabel("time (s)")
    fig.suptitle(f"LoamAir IMU - {len(ts)} samples")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    return png_path


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; background: #14161a; color: #d7dae0;
         font: 13px/1.4 system-ui, Segoe UI, sans-serif; }
  header { display: flex; gap: 12px; align-items: center; padding: 8px 12px;
           border-bottom: 1px solid #2a2e36; flex-wrap: wrap; }
  header h1 { font-size: 14px; margin: 0; font-weight: 600; }
  header .hint { color: #8b919c; }
  button { background: #23272f; color: #d7dae0; border: 1px solid #3a3f49;
           border-radius: 4px; padding: 3px 10px; cursor: pointer; }
  button:hover { background: #2c313a; }
  #readout { margin-left: auto; font-variant-numeric: tabular-nums; color: #9aa1ad; }
  canvas { display: block; width: 100%; height: calc(100vh - 49px); cursor: crosshair; }
</style>
</head>
<body>
<header>
  <h1>__TITLE__</h1>
  <button id="reset">reset zoom</button>
  <span class="hint">wheel = zoom &nbsp;&middot;&nbsp; drag = pan &nbsp;&middot;&nbsp; double-click = reset</span>
  <span id="readout"></span>
</header>
<canvas id="c"></canvas>
<script>
"use strict";
const DATA = __DATA__;
const T = DATA.t, S = DATA.s, LAB = DATA.labels, UN = DATA.units;
const GAP = DATA.gap;
const COLORS = ["#ff6b6b","#51cf66","#4dabf7","#ffa94d","#cc5de8","#22d3ee"];
const N = T.length;
const M = {l: 60, r: 14, t: 10, b: 26};

const canvas = document.getElementById("c");
const ctx = canvas.getContext("2d");
const readout = document.getElementById("readout");
let W = 0, H = 0;

const full = {x0: T[0], x1: T[N - 1]};
if (full.x1 <= full.x0) full.x1 = full.x0 + 1e-3;
let view = {...full};
let mouse = null, drag = null;

function resize() {
  const dpr = window.devicePixelRatio || 1;
  W = canvas.clientWidth; H = canvas.clientHeight;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}

function lowerBound(x) {
  let lo = 0, hi = N;
  while (lo < hi) { const m = (lo + hi) >> 1; if (T[m] < x) lo = m + 1; else hi = m; }
  return lo;
}
function nearestIndex(x) {
  const i = lowerBound(x);
  if (i <= 0) return 0;
  if (i >= N) return N - 1;
  return (x - T[i - 1] <= T[i] - x) ? i - 1 : i;
}

const plotW = () => W - M.l - M.r;
const X = t => M.l + (t - view.x0) / (view.x1 - view.x0) * plotW();

function draw() {
  ctx.clearRect(0, 0, W, H);
  const panelH = (H - M.t - M.b) / 6;
  const i0 = Math.max(0, lowerBound(view.x0) - 1);
  const i1 = Math.min(N - 1, lowerBound(view.x1));

  for (let a = 0; a < 6; a++) {
    const y0 = M.t + a * panelH;
    let mn = Infinity, mx = -Infinity;
    for (let i = i0; i <= i1; i++) { const v = S[a][i]; if (v < mn) mn = v; if (v > mx) mx = v; }
    if (!isFinite(mn)) { mn = -1; mx = 1; }
    if (mx - mn < 1e-6) { const c = (mn + mx) / 2; mn = c - 1; mx = c + 1; }
    const pad = (mx - mn) * 0.08; mn -= pad; mx += pad;
    const inner = panelH - 12;
    const Y = v => y0 + 6 + (mx - v) / (mx - mn) * inner;

    ctx.fillStyle = (a % 2) ? "#171a1f" : "#15181d";
    ctx.fillRect(M.l, y0, plotW(), panelH);

    ctx.strokeStyle = "#262a31"; ctx.lineWidth = 1; ctx.beginPath();
    for (let g = 0; g <= 2; g++) {
      const yy = y0 + 6 + g * inner / 2;
      ctx.moveTo(M.l, yy); ctx.lineTo(W - M.r, yy);
    }
    ctx.stroke();

    ctx.fillStyle = "#8b919c"; ctx.font = "11px system-ui";
    ctx.textAlign = "right"; ctx.textBaseline = "middle";
    ctx.fillText(mx.toFixed(2), M.l - 6, y0 + 10);
    ctx.fillText(mn.toFixed(2), M.l - 6, y0 + panelH - 10);

    ctx.fillStyle = COLORS[a]; ctx.textAlign = "left"; ctx.textBaseline = "top";
    ctx.fillText(LAB[a] + " (" + UN[a] + ")", M.l + 6, y0 + 4);

    ctx.strokeStyle = COLORS[a]; ctx.lineWidth = 1; ctx.beginPath();
    let pen = false, prev = null;
    for (let i = i0; i <= i1; i++) {
      const t = T[i], v = S[a][i];
      if (prev !== null && t - prev > GAP) pen = false;
      const px = X(t), py = Y(v);
      if (pen) ctx.lineTo(px, py); else { ctx.moveTo(px, py); pen = true; }
      prev = t;
    }
    ctx.stroke();

    ctx.strokeStyle = "#2a2e36"; ctx.strokeRect(M.l + 0.5, y0 + 0.5, plotW() - 1, panelH - 1);
  }

  ctx.fillStyle = "#8b919c"; ctx.font = "11px system-ui";
  ctx.textAlign = "center"; ctx.textBaseline = "top";
  for (let g = 0; g <= 5; g++) {
    const t = view.x0 + g / 5 * (view.x1 - view.x0);
    ctx.fillText(t.toFixed(2), X(t), H - M.b + 6);
  }

  if (mouse !== null && mouse >= M.l && mouse <= W - M.r) {
    const t = view.x0 + (mouse - M.l) / plotW() * (view.x1 - view.x0);
    const i = nearestIndex(t);
    const px = X(T[i]);
    ctx.strokeStyle = "#ffffff55"; ctx.lineWidth = 1; ctx.beginPath();
    ctx.moveTo(px, M.t); ctx.lineTo(px, H - M.b); ctx.stroke();
    readout.textContent = "t=" + T[i].toFixed(3) + "s  " +
      LAB.map((l, a) => l + "=" + S[a][i].toFixed(2)).join("  ");
  } else {
    readout.textContent = "";
  }
}

canvas.addEventListener("wheel", e => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  if (mx < M.l || mx > W - M.r) return;
  const tAt = view.x0 + (mx - M.l) / plotW() * (view.x1 - view.x0);
  const factor = Math.exp(e.deltaY * 0.0015);
  const x0 = tAt - (tAt - view.x0) * factor;
  const x1 = tAt + (view.x1 - tAt) * factor;
  if (x1 - x0 < 1e-4) return;
  view = {x0, x1};
  draw();
}, {passive: false});

canvas.addEventListener("mousedown", e => { drag = {x: e.clientX, view: {...view}}; });
window.addEventListener("mouseup", () => { drag = null; });
canvas.addEventListener("mousemove", e => {
  const rect = canvas.getBoundingClientRect();
  mouse = e.clientX - rect.left;
  if (drag) {
    const dt = (drag.x - e.clientX) / plotW() * (drag.view.x1 - drag.view.x0);
    view = {x0: drag.view.x0 + dt, x1: drag.view.x1 + dt};
  }
  draw();
});
canvas.addEventListener("mouseleave", () => { mouse = null; draw(); });
canvas.addEventListener("dblclick", () => { view = {...full}; draw(); });
document.getElementById("reset").onclick = () => { view = {...full}; draw(); };
window.addEventListener("resize", resize);
resize();
</script>
</body>
</html>
"""


def write_html(ts, series, html_path, title):
    n = len(ts)
    step = 1
    if n > MAX_EMBED_SAMPLES:
        step = (n + MAX_EMBED_SAMPLES - 1) // MAX_EMBED_SAMPLES
        ts = ts[::step]
        series = [s[::step] for s in series]
        print(f"note: decimated {n} -> {len(ts)} samples for display")

    gap = max(_median_dt(ts) * 5.0, 1e-4)
    payload = json.dumps(
        {
            "labels": LABELS,
            "units": UNITS,
            "gap": round(gap, 6),
            "t": [round(v, 4) for v in ts],
            "s": [[round(v, 4 if a < 3 else 2) for v in series[a]] for a in range(6)],
        },
        separators=(",", ":"),
    )
    html = HTML_TEMPLATE.replace("__TITLE__", title).replace("__DATA__", payload)
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html)
    return html_path


def render(bin_path, out_base, force_html=False):
    ts, series = read_records(bin_path)
    if not ts:
        print("no records to plot")
        return None

    title = f"LoamAir IMU - {len(ts)} samples"
    if not force_html:
        png = render_matplotlib(ts, series, out_base + ".png")
        if png:
            print(f"wrote {png}")
            return png

    html = write_html(ts, series, out_base + ".html", title)
    print(f"wrote {html}")
    return html


def decode_offline(bin_path, csv_path):
    with open(bin_path, "rb") as f:
        data = f.read()
    with open(csv_path, "w", newline="") as fc:
        fc.write(CSV_HEADER)
        rows, leftover = decode_into(fc, data)
    if leftover:
        print(f"warning: {len(leftover)} trailing bytes (partial record)", file=sys.stderr)
    print(f"decoded {rows} records -> {csv_path}")
    return rows


def main():
    parser = argparse.ArgumentParser(description="LoamAir IMU client")
    parser.add_argument("--host", default="192.168.4.1", help="device softAP IP")
    parser.add_argument("--port", type=int, default=80)
    parser.add_argument("--out", default=None, help="output prefix (default logs/run_<UTC>)")
    parser.add_argument(
        "--duration", type=float, default=0.0, help="seconds to stream (0 = until Ctrl+C)"
    )
    parser.add_argument("--no-plot", action="store_true", help="skip rendering the plot")
    parser.add_argument(
        "--html", action="store_true", help="force interactive HTML (skip matplotlib)"
    )
    parser.add_argument("--offline", metavar="BIN", default=None, help="re-plot a saved .bin")
    args = parser.parse_args()

    if args.offline:
        bin_path = args.offline
        base = os.path.splitext(bin_path)[0]
        decode_offline(bin_path, base + ".csv")
        if not args.no_plot:
            render(bin_path, base, force_html=args.html)
        return

    out_prefix = args.out
    if out_prefix is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        out_prefix = os.path.join("logs", f"run_{stamp}")

    bin_path, _, total = stream(args.host, args.port, out_prefix, args.duration)
    if not args.no_plot and total:
        render(bin_path, out_prefix, force_html=args.html)


if __name__ == "__main__":
    main()
