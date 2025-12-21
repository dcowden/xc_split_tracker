import sys
import numpy as np
import pandas as pd

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


CSV_PATH = r"c:\gitwork\xc_split_tracker\data\raw-3x3 approaches-12-20-2025.csv"
EMA_PRESETS = (0.01, 0.05, 0.10)


# -------------------------
# Filters
# -------------------------
def ema_np(x: np.ndarray, alpha: float) -> np.ndarray:
    """EMA: y[0]=x[0], y[i]=alpha*x[i]+(1-alpha)*y[i-1]"""
    x = x.astype(np.float32, copy=False)
    y = np.empty_like(x, dtype=np.float32)
    if x.size == 0:
        return y
    y[0] = x[0]
    a = np.float32(alpha)
    om = np.float32(1.0 - alpha)
    for i in range(1, x.size):
        y[i] = a * x[i] + om * y[i - 1]
    return y


def baseline_ema_clamped(x: np.ndarray, alpha: float, clamp_up_db: float) -> np.ndarray:
    """
    Baseline tracker that follows downward/steady quickly, but resists being pulled upward by peaks.
    If x is more than clamp_up_db above baseline, baseline updates are suppressed.
    """
    x = x.astype(np.float32, copy=False)
    n = x.size
    b = np.empty(n, dtype=np.float32)
    if n == 0:
        return b
    a = float(alpha)
    if a <= 0.0:
        b[:] = x[0]
        return b

    base = float(x[0])
    b[0] = base
    for i in range(1, n):
        xi = float(x[i])
        if xi <= base + clamp_up_db:
            base = base + a * (xi - base)
        # else: ignore upward excursions (runner peaks)
        b[i] = base
    return b


def kalman_1d_random_walk(z: np.ndarray, q: float, r: float, x0=None, p0=None) -> np.ndarray:
    """
    Simple 1D Kalman filter (random-walk model):
      x_k = x_{k-1} + w,   w~N(0, q)
      z_k = x_k + v,       v~N(0, r)
    """
    z = z.astype(np.float32, copy=False)
    n = z.size
    xhat = np.empty(n, dtype=np.float32)
    if n == 0:
        return xhat

    x = float(z[0]) if x0 is None else float(x0)
    p = 1.0 if p0 is None else float(p0)

    q = float(max(q, 1e-12))
    r = float(max(r, 1e-12))

    for k in range(n):
        # predict
        p = p + q

        # update
        y = float(z[k]) - x
        s = p + r
        k_gain = p / s
        x = x + k_gain * y
        p = (1.0 - k_gain) * p

        xhat[k] = x

    return xhat


# -------------------------
# Pass detection algorithms
# -------------------------
def _ms_to_samples(t_ms: np.ndarray, window_ms: float) -> int:
    """Approx convert ms->samples using median dt (robust enough)."""
    if t_ms.size < 2:
        return 1
    dt = np.diff(t_ms.astype(np.int64))
    dt = dt[dt > 0]
    if dt.size == 0:
        return 1
    med = float(np.median(dt))
    if med <= 0:
        return 1
    return max(1, int(round(window_ms / med)))


def passes_opt1_baseline_hysteresis(
    t_ms: np.ndarray,
    x: np.ndarray,
    base_alpha: float,
    clamp_up_db: float,
    delta_on_db: float,
    delta_off_db: float,
    min_dur_ms: int,
    refractory_ms: int,
):
    """
    Option 1: baseline + hysteresis
      baseline = clamped EMA (won't chase peaks)
      start when x > baseline + delta_on
      end when x < baseline + delta_off
    """
    n = x.size
    if n == 0:
        return [], None

    b = baseline_ema_clamped(x, base_alpha, clamp_up_db=clamp_up_db)
    on_thr = b + float(delta_on_db)
    off_thr = b + float(delta_off_db)

    starts_ends = []
    in_event = False
    start_i = 0

    refractory_until_ms = -10**18

    for i in range(n):
        ti = int(t_ms[i])

        if ti < refractory_until_ms:
            continue

        if not in_event:
            if float(x[i]) > float(on_thr[i]):
                in_event = True
                start_i = i
        else:
            # end condition
            if float(x[i]) < float(off_thr[i]):
                end_i = i
                # enforce min duration
                if int(t_ms[end_i]) - int(t_ms[start_i]) >= int(min_dur_ms):
                    starts_ends.append((start_i, end_i))
                    refractory_until_ms = int(t_ms[end_i]) + int(refractory_ms)
                in_event = False

    return starts_ends, b


def _local_maxima(y: np.ndarray) -> np.ndarray:
    """Indices i where y[i-1] < y[i] >= y[i+1]."""
    if y.size < 3:
        return np.array([], dtype=np.int64)
    return np.where((y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:]))[0] + 1


def _peak_prominence_approx(y: np.ndarray, peak_idx: int, w: int) -> float:
    """
    Approx prominence: peak height minus max of local minima on left/right within window w.
    """
    n = y.size
    i = peak_idx
    lo = max(0, i - w)
    hi = min(n - 1, i + w)

    left_min = float(np.min(y[lo:i + 1])) if i > lo else float(y[i])
    right_min = float(np.min(y[i:hi + 1])) if hi > i else float(y[i])

    ref = max(left_min, right_min)
    return float(y[i]) - ref


def passes_opt2_detrended_peaks(
    t_ms: np.ndarray,
    x: np.ndarray,
    base_alpha: float,
    clamp_up_db: float,
    prom_window_ms: int,
    prom_min_db: float,
    min_peak_dist_ms: int,
    end_thresh_db: float,
):
    """
    Option 2: detrended peak detection
      baseline = clamped EMA
      y = x - baseline
      find peaks with prominence >= prom_min_db and separated by min_peak_dist_ms
      for each peak, define start/end by y falling below end_thresh_db
    """
    n = x.size
    if n == 0:
        return [], None, np.array([], dtype=np.int64)

    b = baseline_ema_clamped(x, base_alpha, clamp_up_db=clamp_up_db)
    y = x.astype(np.float32) - b

    w_samp = _ms_to_samples(t_ms, prom_window_ms)

    peaks = _local_maxima(y)
    if peaks.size == 0:
        return [], b, peaks

    # compute prominence and filter
    proms = np.array([_peak_prominence_approx(y, int(pi), w_samp) for pi in peaks], dtype=np.float32)
    keep = proms >= float(prom_min_db)
    peaks = peaks[keep]
    proms = proms[keep]

    if peaks.size == 0:
        return [], b, peaks

    # enforce min distance in time (greedy by prominence desc)
    order = np.argsort(-proms)  # highest prominence first
    selected = []
    min_dist = int(min_peak_dist_ms)
    for idx in order:
        p = int(peaks[idx])
        tp = int(t_ms[p])
        ok = True
        for q in selected:
            if abs(tp - int(t_ms[q])) < min_dist:
                ok = False
                break
        if ok:
            selected.append(p)

    selected = np.array(sorted(selected), dtype=np.int64)
    if selected.size == 0:
        return [], b, selected

    # segment start/end around each selected peak using y threshold
    starts_ends = []
    thr = float(end_thresh_db)

    for j, p in enumerate(selected):
        # left boundary: walk left until y < thr
        i0 = int(p)
        while i0 > 0 and float(y[i0]) >= thr:
            i0 -= 1
        start_i = i0

        # right boundary: walk right until y < thr
        i1 = int(p)
        while i1 < n - 1 and float(y[i1]) >= thr:
            i1 += 1
        end_i = i1

        # Avoid overlaps by clipping boundaries to midpoints between peaks
        if j > 0:
            mid_left = (int(selected[j - 1]) + int(p)) // 2
            start_i = max(start_i, mid_left)
        if j < selected.size - 1:
            mid_right = (int(p) + int(selected[j + 1])) // 2
            end_i = min(end_i, mid_right)

        if end_i > start_i:
            starts_ends.append((start_i, end_i))

    return starts_ends, b, selected


def passes_opt3_derivative(
    t_ms: np.ndarray,
    x: np.ndarray,
    smooth_alpha: float,
    base_alpha: float,
    clamp_up_db: float,
    slope_on_db_per_s: float,
    slope_off_db_per_s: float,
    height_on_db: float,
    height_off_db: float,
    min_dur_ms: int,
    refractory_ms: int,
):
    """
    Option 3: derivative/slope segmentation
      xs = EMA(x, smooth_alpha)
      baseline = clamped EMA(xs, base_alpha)
      y = xs - baseline
      slope = d(xs)/dt in dB/s
      start when slope > slope_on AND y > height_on
      end when slope < -slope_off AND y < height_off  (plus min_dur + refractory)
    """
    n = x.size
    if n < 3:
        return [], None, None

    xs = ema_np(x, smooth_alpha)
    b = baseline_ema_clamped(xs, base_alpha, clamp_up_db=clamp_up_db)
    y = xs - b

    dt_ms = np.diff(t_ms.astype(np.int64))
    dt_ms[dt_ms <= 0] = 1
    dx = np.diff(xs.astype(np.float32))
    slope = (dx / dt_ms.astype(np.float32)) * 1000.0  # dB/s

    print("opt3 debug:",
          "dt_ms median", float(np.median(dt_ms)),
          "slope min/max", float(np.min(slope)), float(np.max(slope)),
          "y min/max", float(np.min(y)), float(np.max(y)))

    starts_ends = []
    in_event = False
    start_i = 0
    refractory_until_ms = -10**18

    for i in range(1, n - 1):
        ti = int(t_ms[i])
        if ti < refractory_until_ms:
            continue

        s = float(slope[i - 1])  # slope for segment (i-1 -> i)

        if not in_event:
            if s > float(slope_on_db_per_s) and float(y[i]) > float(height_on_db):
                in_event = True
                start_i = i
        else:
            # end rule: falling slope AND back near baseline
            if s < -float(slope_off_db_per_s) and float(y[i]) < float(height_off_db):
                end_i = i
                if int(t_ms[end_i]) - int(t_ms[start_i]) >= int(min_dur_ms):
                    starts_ends.append((start_i, end_i))
                    refractory_until_ms = int(t_ms[end_i]) + int(refractory_ms)
                in_event = False

    return starts_ends, b, xs


# -------------------------
# CSV load
# -------------------------
def load_csv_rel_ms(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, engine="c")
    df.columns = [c.strip() for c in df.columns]

    required = {"rel_ms", "rssi", "tag_id", "pass_id"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Missing required columns: {sorted(missing)}")

    for col in ["rel_ms", "rssi", "tag_id", "pass_id"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=["rel_ms", "rssi", "tag_id", "pass_id"])

    df["rel_ms"] = df["rel_ms"].astype(np.int64)
    df["rssi"] = df["rssi"].astype(np.float32)
    df["tag_id"] = df["tag_id"].astype(np.int64)
    df["pass_id"] = df["pass_id"].astype(np.int64)

    if "unix_ms" in df.columns:
        df["unix_ms"] = pd.to_numeric(df["unix_ms"], errors="coerce").astype("Int64")

    df = df.sort_values("rel_ms", kind="mergesort")
    return df


# -------------------------
# App
# -------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, csv_path: str):
        super().__init__()
        self.setWindowTitle("RSSI Plot (pyqtgraph) — rel_ms + EMA + Kalman + pass detection (3 options)")
        self.resize(1900, 980)

        self.df = load_csv_rel_ms(csv_path)

        self.t_ms = np.array([], dtype=np.int64)
        self.rssi = np.array([], dtype=np.float32)

        self.ema_items = []
        self.kalman_item = None

        self.raw_header = "unix_ms,rel_ms,tag_id,pass_id,rssi"
        self.raw_row_offset = 1

        # --- pass marker items (square markers) ---
        # --- pass marker items: start=square, end=circle ---
        # Option 1 (red)
        self.opt1_start = pg.ScatterPlotItem(symbol="s", size=9,
                                             pen=pg.mkPen((255, 0, 0, 220)),
                                             brush=pg.mkBrush((255, 0, 0, 160)))
        self.opt1_end = pg.ScatterPlotItem(symbol="o", size=9,
                                           pen=pg.mkPen((255, 0, 0, 220)),
                                           brush=pg.mkBrush((255, 0, 0, 160)))

        # Option 2 (blue)
        self.opt2_start = pg.ScatterPlotItem(symbol="s", size=9,
                                             pen=pg.mkPen((0, 90, 255, 220)),
                                             brush=pg.mkBrush((0, 90, 255, 140)))
        self.opt2_end = pg.ScatterPlotItem(symbol="o", size=9,
                                           pen=pg.mkPen((0, 90, 255, 220)),
                                           brush=pg.mkBrush((0, 90, 255, 140)))

        # Option 3 (green)
        self.opt3_start = pg.ScatterPlotItem(symbol="s", size=9,
                                             pen=pg.mkPen((0, 170, 0, 220)),
                                             brush=pg.mkBrush((0, 170, 0, 140)))
        self.opt3_end = pg.ScatterPlotItem(symbol="o", size=9,
                                           pen=pg.mkPen((0, 170, 0, 220)),
                                           brush=pg.mkBrush((0, 170, 0, 140)))

        # ---- UI ----
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Controls (multi-line friendly)
        controls = QtWidgets.QGridLayout()
        controls.setHorizontalSpacing(10)
        controls.setVerticalSpacing(6)

        # Row 0: filters
        self.tag_combo = QtWidgets.QComboBox()
        self.pass_combo = QtWidgets.QComboBox()

        self.tag_combo.addItem("(all)")
        for t in sorted(self.df["tag_id"].unique().tolist()):
            self.tag_combo.addItem(str(t))

        self.pass_combo.addItem("(all)")
        for p in sorted(self.df["pass_id"].unique().tolist()):
            self.pass_combo.addItem(str(p))

        controls.addWidget(QtWidgets.QLabel("tag_id:"), 0, 0)
        controls.addWidget(self.tag_combo, 0, 1)
        controls.addWidget(QtWidgets.QLabel("pass_id:"), 0, 2)
        controls.addWidget(self.pass_combo, 0, 3)

        # Row 0: status right
        self.status = QtWidgets.QLabel("")
        self.status.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
        controls.addWidget(self.status, 0, 9, 1, 3)

        # Row 1: EMA + Kalman
        controls.addWidget(QtWidgets.QLabel("EMA presets:"), 1, 0)
        self.preset_checks = []
        col = 1
        for a in EMA_PRESETS:
            cb = QtWidgets.QCheckBox(f"{a:g}")
            cb.setChecked(True)
            self.preset_checks.append(cb)
            controls.addWidget(cb, 1, col)
            col += 1

        controls.addWidget(QtWidgets.QLabel("custom α:"), 1, col)
        col += 1
        self.custom_alpha = QtWidgets.QLineEdit()
        self.custom_alpha.setPlaceholderText("e.g. 0.07")
        self.custom_alpha.setFixedWidth(70)
        controls.addWidget(self.custom_alpha, 1, col)
        col += 1

        controls.addWidget(QtWidgets.QLabel("Kalman enable:"), 1, col)
        col += 1
        self.kalman_enable = QtWidgets.QCheckBox()
        self.kalman_enable.setChecked(True)
        controls.addWidget(self.kalman_enable, 1, col)
        col += 1

        controls.addWidget(QtWidgets.QLabel("q:"), 1, col)
        col += 1
        self.kalman_q = QtWidgets.QDoubleSpinBox()
        self.kalman_q.setDecimals(9)
        self.kalman_q.setRange(0.0, 1e6)
        self.kalman_q.setSingleStep(1e-6)
        self.kalman_q.setValue(0.000004)
        self.kalman_q.setFixedWidth(120)
        controls.addWidget(self.kalman_q, 1, col)
        col += 1

        controls.addWidget(QtWidgets.QLabel("r:"), 1, col)
        col += 1
        self.kalman_r = QtWidgets.QDoubleSpinBox()
        self.kalman_r.setDecimals(6)
        self.kalman_r.setRange(0.0, 1e6)
        self.kalman_r.setSingleStep(0.01)
        self.kalman_r.setValue(0.1)
        self.kalman_r.setFixedWidth(90)
        controls.addWidget(self.kalman_r, 1, col)

        # Row 2: Option 1 params
        r = 2
        controls.addWidget(QtWidgets.QLabel("Opt1 (red) baseline+HYS:"), r, 0)

        self.o1_base_alpha = QtWidgets.QDoubleSpinBox()
        self.o1_base_alpha.setDecimals(6)
        self.o1_base_alpha.setRange(0.0, 1.0)
        self.o1_base_alpha.setSingleStep(0.0005)
        self.o1_base_alpha.setValue(0.002)
        self.o1_base_alpha.setFixedWidth(90)

        self.o1_clamp = QtWidgets.QDoubleSpinBox()
        self.o1_clamp.setDecimals(2)
        self.o1_clamp.setRange(0.0, 50.0)
        self.o1_clamp.setSingleStep(0.5)
        self.o1_clamp.setValue(2.0)
        self.o1_clamp.setFixedWidth(70)

        self.o1_on = QtWidgets.QDoubleSpinBox()
        self.o1_on.setDecimals(2)
        self.o1_on.setRange(0.0, 50.0)
        self.o1_on.setSingleStep(0.5)
        self.o1_on.setValue(4.0)
        self.o1_on.setFixedWidth(70)

        self.o1_off = QtWidgets.QDoubleSpinBox()
        self.o1_off.setDecimals(2)
        self.o1_off.setRange(0.0, 50.0)
        self.o1_off.setSingleStep(0.5)
        self.o1_off.setValue(2.0)
        self.o1_off.setFixedWidth(70)

        self.o1_min_dur = QtWidgets.QSpinBox()
        self.o1_min_dur.setRange(0, 600000)
        self.o1_min_dur.setSingleStep(10)
        self.o1_min_dur.setValue(120)
        self.o1_min_dur.setFixedWidth(80)

        self.o1_refrac = QtWidgets.QSpinBox()
        self.o1_refrac.setRange(0, 600000)
        self.o1_refrac.setSingleStep(50)
        self.o1_refrac.setValue(250)
        self.o1_refrac.setFixedWidth(80)

        c = 1
        controls.addWidget(QtWidgets.QLabel("base α"), r, c); c += 1
        controls.addWidget(self.o1_base_alpha, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("clamp↑dB"), r, c); c += 1
        controls.addWidget(self.o1_clamp, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("Δon"), r, c); c += 1
        controls.addWidget(self.o1_on, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("Δoff"), r, c); c += 1
        controls.addWidget(self.o1_off, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("min ms"), r, c); c += 1
        controls.addWidget(self.o1_min_dur, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("refrac ms"), r, c); c += 1
        controls.addWidget(self.o1_refrac, r, c)

        # Row 3: Option 2 params
        r = 3
        controls.addWidget(QtWidgets.QLabel("Opt2 (blue) detrend+peaks:"), r, 0)

        self.o2_base_alpha = QtWidgets.QDoubleSpinBox()
        self.o2_base_alpha.setDecimals(6)
        self.o2_base_alpha.setRange(0.0, 1.0)
        self.o2_base_alpha.setSingleStep(0.0005)
        self.o2_base_alpha.setValue(0.002)
        self.o2_base_alpha.setFixedWidth(90)

        self.o2_clamp = QtWidgets.QDoubleSpinBox()
        self.o2_clamp.setDecimals(2)
        self.o2_clamp.setRange(0.0, 50.0)
        self.o2_clamp.setSingleStep(0.5)
        self.o2_clamp.setValue(2.0)
        self.o2_clamp.setFixedWidth(70)

        self.o2_prom_win = QtWidgets.QSpinBox()
        self.o2_prom_win.setRange(10, 600000)
        self.o2_prom_win.setSingleStep(50)
        self.o2_prom_win.setValue(600)  # ms
        self.o2_prom_win.setFixedWidth(80)

        self.o2_prom_min = QtWidgets.QDoubleSpinBox()
        self.o2_prom_min.setDecimals(2)
        self.o2_prom_min.setRange(0.0, 50.0)
        self.o2_prom_min.setSingleStep(0.5)
        self.o2_prom_min.setValue(5.0)
        self.o2_prom_min.setFixedWidth(70)

        self.o2_dist = QtWidgets.QSpinBox()
        self.o2_dist.setRange(0, 600000)
        self.o2_dist.setSingleStep(50)
        self.o2_dist.setValue(400)
        self.o2_dist.setFixedWidth(80)

        self.o2_end_thr = QtWidgets.QDoubleSpinBox()
        self.o2_end_thr.setDecimals(2)
        self.o2_end_thr.setRange(-50.0, 50.0)
        self.o2_end_thr.setSingleStep(0.5)
        self.o2_end_thr.setValue(1.5)
        self.o2_end_thr.setFixedWidth(70)

        c = 1
        controls.addWidget(QtWidgets.QLabel("base α"), r, c); c += 1
        controls.addWidget(self.o2_base_alpha, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("clamp↑dB"), r, c); c += 1
        controls.addWidget(self.o2_clamp, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("prom win ms"), r, c); c += 1
        controls.addWidget(self.o2_prom_win, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("prom min"), r, c); c += 1
        controls.addWidget(self.o2_prom_min, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("min dist ms"), r, c); c += 1
        controls.addWidget(self.o2_dist, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("end thr"), r, c); c += 1
        controls.addWidget(self.o2_end_thr, r, c)

        # Row 4: Option 3 params
        r = 4
        controls.addWidget(QtWidgets.QLabel("Opt3 (green) derivative:"), r, 0)

        self.o3_smooth_alpha = QtWidgets.QDoubleSpinBox()
        self.o3_smooth_alpha.setDecimals(4)
        self.o3_smooth_alpha.setRange(0.0, 1.0)
        self.o3_smooth_alpha.setSingleStep(0.01)
        self.o3_smooth_alpha.setValue(0.1)
        self.o3_smooth_alpha.setFixedWidth(80)

        self.o3_base_alpha = QtWidgets.QDoubleSpinBox()
        self.o3_base_alpha.setDecimals(6)
        self.o3_base_alpha.setRange(0.0, 1.0)
        self.o3_base_alpha.setSingleStep(0.0005)
        self.o3_base_alpha.setValue(0.002)
        self.o3_base_alpha.setFixedWidth(90)

        self.o3_clamp = QtWidgets.QDoubleSpinBox()
        self.o3_clamp.setDecimals(2)
        self.o3_clamp.setRange(0.0, 50.0)
        self.o3_clamp.setSingleStep(0.5)
        self.o3_clamp.setValue(2.0)
        self.o3_clamp.setFixedWidth(70)

        self.o3_slope_on = QtWidgets.QDoubleSpinBox()
        self.o3_slope_on.setDecimals(1)
        self.o3_slope_on.setRange(0.0, 5000.0)
        self.o3_slope_on.setSingleStep(50.0)
        self.o3_slope_on.setValue(300.0)
        self.o3_slope_on.setFixedWidth(90)

        self.o3_slope_off = QtWidgets.QDoubleSpinBox()
        self.o3_slope_off.setDecimals(1)
        self.o3_slope_off.setRange(0.0, 5000.0)
        self.o3_slope_off.setSingleStep(50.0)
        self.o3_slope_off.setValue(300.0)
        self.o3_slope_off.setFixedWidth(90)

        self.o3_h_on = QtWidgets.QDoubleSpinBox()
        self.o3_h_on.setDecimals(2)
        self.o3_h_on.setRange(0.0, 50.0)
        self.o3_h_on.setSingleStep(0.5)
        self.o3_h_on.setValue(2.0)
        self.o3_h_on.setFixedWidth(70)

        self.o3_h_off = QtWidgets.QDoubleSpinBox()
        self.o3_h_off.setDecimals(2)
        self.o3_h_off.setRange(0.0, 50.0)
        self.o3_h_off.setSingleStep(0.5)
        self.o3_h_off.setValue(1.5)
        self.o3_h_off.setFixedWidth(70)

        self.o3_min_dur = QtWidgets.QSpinBox()
        self.o3_min_dur.setRange(0, 600000)
        self.o3_min_dur.setSingleStep(10)
        self.o3_min_dur.setValue(120)
        self.o3_min_dur.setFixedWidth(80)

        self.o3_refrac = QtWidgets.QSpinBox()
        self.o3_refrac.setRange(0, 600000)
        self.o3_refrac.setSingleStep(50)
        self.o3_refrac.setValue(250)
        self.o3_refrac.setFixedWidth(80)

        c = 1
        controls.addWidget(QtWidgets.QLabel("smooth α"), r, c); c += 1
        controls.addWidget(self.o3_smooth_alpha, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("base α"), r, c); c += 1
        controls.addWidget(self.o3_base_alpha, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("clamp↑dB"), r, c); c += 1
        controls.addWidget(self.o3_clamp, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("slope on dB/s"), r, c); c += 1
        controls.addWidget(self.o3_slope_on, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("slope off dB/s"), r, c); c += 1
        controls.addWidget(self.o3_slope_off, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("h on"), r, c); c += 1
        controls.addWidget(self.o3_h_on, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("h off"), r, c); c += 1
        controls.addWidget(self.o3_h_off, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("min ms"), r, c); c += 1
        controls.addWidget(self.o3_min_dur, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("refrac ms"), r, c); c += 1
        controls.addWidget(self.o3_refrac, r, c)

        # Row 5: Apply
        self.apply_btn = QtWidgets.QPushButton("Apply overlays")
        controls.addWidget(self.apply_btn, 5, 0, 1, 2)

        root.addLayout(controls)

        # Splitter: plot + raw panel
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        root.addWidget(splitter, 1)

        pg.setConfigOptions(antialias=False)
        self.win = pg.GraphicsLayoutWidget()
        splitter.addWidget(self.win)

        raw_panel = QtWidgets.QWidget()
        raw_layout = QtWidgets.QVBoxLayout(raw_panel)
        raw_layout.setContentsMargins(6, 6, 6, 6)
        raw_layout.setSpacing(6)
        raw_layout.addWidget(QtWidgets.QLabel("Raw data (click a point to select row):"))

        self.raw_list = QtWidgets.QListWidget()
        self.raw_list.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.raw_list.setAlternatingRowColors(True)
        self.raw_list.setStyleSheet("""
            QListWidget::item:selected { background: #2b6cb0; color: white; }
        """)
        raw_layout.addWidget(self.raw_list, 1)
        splitter.addWidget(raw_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        # Plot
        self.plot = self.win.addPlot(row=0, col=0)
        self.plot.setLabel("bottom", "rel_ms", units="ms")
        self.plot.setLabel("left", "RSSI", units="dBm")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setMouseEnabled(x=True, y=True)
        self.axis = self.plot.getAxis("bottom")

        # Raw series: line + markers
        self.raw_line = pg.PlotDataItem(pen=pg.mkPen((110, 130, 150, 120), width=1))
        self.raw_line.setDownsampling(auto=True, method="peak")
        self.raw_line.setClipToView(True)
        self.plot.addItem(self.raw_line)

        self.raw_scatter = pg.ScatterPlotItem(
            pen=pg.mkPen((110, 130, 150, 140)),
            brush=pg.mkBrush((110, 130, 150, 60)),
            symbol="o",
            size=5,
            pxMode=True,
        )
        self.plot.addItem(self.raw_scatter)

        # Add pass markers (above everything)
        self.plot.addItem(self.opt1_start)
        self.plot.addItem(self.opt1_end)
        self.plot.addItem(self.opt2_start)
        self.plot.addItem(self.opt2_end)
        self.plot.addItem(self.opt3_start)
        self.plot.addItem(self.opt3_end)

        # Clicking points -> highlight row
        self.raw_scatter.sigClicked.connect(self.on_points_clicked)

        # Wire signals
        self.apply_btn.clicked.connect(self.recompute_overlays)
        self.custom_alpha.returnPressed.connect(self.recompute_overlays)
        self.tag_combo.currentIndexChanged.connect(self.on_filter_changed)
        self.pass_combo.currentIndexChanged.connect(self.on_filter_changed)

        for cb in self.preset_checks:
            cb.stateChanged.connect(self.recompute_overlays)

        self.kalman_enable.stateChanged.connect(self.recompute_overlays)
        self.kalman_q.valueChanged.connect(self.recompute_overlays)
        self.kalman_r.valueChanged.connect(self.recompute_overlays)

        # Pass params -> live recompute
        for w in [
            self.o1_base_alpha, self.o1_clamp, self.o1_on, self.o1_off, self.o1_min_dur, self.o1_refrac,
            self.o2_base_alpha, self.o2_clamp, self.o2_prom_win, self.o2_prom_min, self.o2_dist, self.o2_end_thr,
            self.o3_smooth_alpha, self.o3_base_alpha, self.o3_clamp, self.o3_slope_on, self.o3_slope_off,
            self.o3_h_on, self.o3_h_off, self.o3_min_dur, self.o3_refrac,
        ]:
            if isinstance(w, QtWidgets.QSpinBox) or isinstance(w, QtWidgets.QDoubleSpinBox):
                w.valueChanged.connect(self.recompute_overlays)

        # Initial
        self.apply_filters_and_plot_raw()
        self.recompute_overlays()

    # -------------------------
    # Raw panel linking
    # -------------------------
    def highlight_row(self, idx: int):
        row = int(idx) + self.raw_row_offset
        if 0 <= row < self.raw_list.count():
            self.raw_list.setCurrentRow(row)
            item = self.raw_list.item(row)
            self.raw_list.scrollToItem(item, QtWidgets.QAbstractItemView.ScrollHint.PositionAtCenter)

    def on_points_clicked(self, plot_item, points):
        if points is None:
            return
        try:
            n = len(points)
        except Exception:
            return
        if n == 0:
            return
        p0 = points[0]
        try:
            idx = p0.data()
        except Exception:
            idx = None
        if idx is None:
            return
        self.highlight_row(int(idx))

    # -------------------------
    # Data + plotting
    # -------------------------
    def on_filter_changed(self):
        self.apply_filters_and_plot_raw()
        self.recompute_overlays()

    def apply_filters_and_plot_raw(self):
        df = self.df

        tag_text = self.tag_combo.currentText()
        pass_text = self.pass_combo.currentText()

        if tag_text != "(all)":
            df = df[df["tag_id"] == int(tag_text)]
        if pass_text != "(all)":
            df = df[df["pass_id"] == int(pass_text)]

        if df.empty:
            self.status.setText("No data after filtering")
            self.t_ms = np.array([], dtype=np.int64)
            self.rssi = np.array([], dtype=np.float32)
            self.raw_scatter.setData([], [])
            self.raw_line.setData([], [])
            self.raw_list.clear()
            return

        t_ms = df["rel_ms"].to_numpy(dtype=np.int64)
        rssi = df["rssi"].to_numpy(dtype=np.float32)
        self.t_ms = t_ms
        self.rssi = rssi

        self.raw_line.setData(t_ms, rssi)
        idx = np.arange(len(t_ms), dtype=np.int32)
        self.raw_scatter.setData(x=t_ms, y=rssi, data=idx)

        # Raw panel
        self.raw_list.clear()
        self.raw_list.addItem(self.raw_header)

        has_unix = "unix_ms" in df.columns
        unix_vals = df["unix_ms"].to_numpy(dtype="int64", na_value=-1) if has_unix else None

        rel_vals = t_ms
        tag_vals = df["tag_id"].to_numpy(dtype=np.int64)
        pass_vals = df["pass_id"].to_numpy(dtype=np.int64)
        rssi_vals = rssi

        for i in range(len(rel_vals)):
            u_str = ""
            if has_unix:
                u = unix_vals[i]
                u_str = "" if u == -1 else str(int(u))
            line = f"{u_str},{int(rel_vals[i])},{int(tag_vals[i])},{int(pass_vals[i])},{int(round(float(rssi_vals[i])))}"
            self.raw_list.addItem(line)

        # X ticks: major 1000ms, minor 100ms
        x0 = float(t_ms[0])
        x1 = float(t_ms[-1])

        major_start = np.floor(x0 / 1000.0) * 1000.0
        major_end = np.ceil(x1 / 1000.0) * 1000.0
        major_vals = np.arange(major_start, major_end + 1000.0, 1000.0)

        minor_start = np.floor(x0 / 100.0) * 100.0
        minor_end = np.ceil(x1 / 100.0) * 100.0
        minor_vals = np.arange(minor_start, minor_end + 100.0, 100.0)

        major_ticks = [(float(v), f"{int(v)}") for v in major_vals]
        minor_ticks = [(float(v), "") for v in minor_vals]
        self.axis.setTicks([major_ticks, minor_ticks])

        self.plot.setXRange(x0, x1, padding=0.01)
        self.status.setText(f"{len(t_ms):,} samples  (rel_ms {int(x0)}..{int(x1)})")

    # -------------------------
    # Overlays
    # -------------------------
    def get_selected_alphas(self):
        alphas = []
        for cb in self.preset_checks:
            if cb.isChecked():
                try:
                    alphas.append(float(cb.text()))
                except ValueError:
                    pass

        s = self.custom_alpha.text().strip()
        if s:
            try:
                a = float(s)
                if 0.0 < a <= 1.0:
                    alphas.append(a)
            except ValueError:
                pass

        return sorted(set(alphas))

    def clear_ema_items(self):
        for item in self.ema_items:
            try:
                self.plot.removeItem(item)
            except Exception:
                pass
        self.ema_items = []

    def clear_kalman_item(self):
        if self.kalman_item is not None:
            try:
                self.plot.removeItem(self.kalman_item)
            except Exception:
                pass
            self.kalman_item = None

    def set_pass_markers(self, passes, start_item, end_item, y_series=None):
        if y_series is None:
            y_series = self.rssi

        if not passes:
            start_item.setData([], [])
            end_item.setData([], [])
            return

        xs_s, ys_s = [], []
        xs_e, ys_e = [], []
        n = self.t_ms.size

        for s, e in passes:
            s = int(s);
            e = int(e)
            if 0 <= s < n:
                xs_s.append(float(self.t_ms[s]));
                ys_s.append(float(y_series[s]))
            if 0 <= e < n:
                xs_e.append(float(self.t_ms[e]));
                ys_e.append(float(y_series[e]))

        start_item.setData(xs_s, ys_s)
        end_item.setData(xs_e, ys_e)

    def recompute_overlays(self):
        if self.t_ms is None or self.rssi is None or self.t_ms.size == 0:
            return

        # -------------------------
        # EMA overlays (on raw RSSI)
        # -------------------------
        self.clear_ema_items()
        alphas = self.get_selected_alphas()
        if alphas:
            pens = [
                pg.mkPen((255, 165, 0, 220), width=2),
                pg.mkPen((50, 205, 50, 220), width=2),
                pg.mkPen((220, 20, 60, 220), width=2),
                pg.mkPen((30, 144, 255, 220), width=2),
                pg.mkPen((148, 0, 211, 220), width=2),
                pg.mkPen((0, 206, 209, 220), width=2),
            ]
            for i, a in enumerate(alphas):
                y = ema_np(self.rssi, a)
                curve = self.plot.plot(self.t_ms, y, pen=pens[i % len(pens)])
                curve.setDownsampling(auto=True, method="peak")
                curve.setClipToView(True)
                self.ema_items.append(curve)

        # -------------------------
        # Kalman overlay
        # -------------------------
        self.clear_kalman_item()

        yk = None
        if self.kalman_enable.isChecked():
            q = float(self.kalman_q.value())
            r = float(self.kalman_r.value())
            yk = kalman_1d_random_walk(self.rssi, q=q, r=r)

            self.kalman_item = self.plot.plot(
                self.t_ms,
                yk,
                pen=pg.mkPen((148, 0, 211, 230), width=3),  # purple
            )
            self.kalman_item.setDownsampling(auto=True, method="peak")
            self.kalman_item.setClipToView(True)

        # Use Kalman-smoothed data for detection if available; otherwise fall back to raw
        x_det = yk if yk is not None else self.rssi

        # -------------------------
        # Pass detection (3 options) on x_det
        # Markers are positioned using x_det so they sit on the smooth curve.
        # -------------------------
        opt1, _b1 = passes_opt1_baseline_hysteresis(
            self.t_ms, x_det,
            base_alpha=float(self.o1_base_alpha.value()),
            clamp_up_db=float(self.o1_clamp.value()),
            delta_on_db=float(self.o1_on.value()),
            delta_off_db=float(self.o1_off.value()),
            min_dur_ms=int(self.o1_min_dur.value()),
            refractory_ms=int(self.o1_refrac.value()),
        )
        self.set_pass_markers(opt1, self.opt1_start, self.opt1_end, y_series=x_det)

        opt2, _b2, _peaks = passes_opt2_detrended_peaks(
            self.t_ms, x_det,
            base_alpha=float(self.o2_base_alpha.value()),
            clamp_up_db=float(self.o2_clamp.value()),
            prom_window_ms=int(self.o2_prom_win.value()),
            prom_min_db=float(self.o2_prom_min.value()),
            min_peak_dist_ms=int(self.o2_dist.value()),
            end_thresh_db=float(self.o2_end_thr.value()),
        )
        self.set_pass_markers(opt2, self.opt2_start, self.opt2_end, y_series=x_det)

        opt3, _b3, _xs = passes_opt3_derivative(
            self.t_ms, x_det,
            smooth_alpha=float(self.o3_smooth_alpha.value()),
            base_alpha=float(self.o3_base_alpha.value()),
            clamp_up_db=float(self.o3_clamp.value()),
            slope_on_db_per_s=float(self.o3_slope_on.value()),
            slope_off_db_per_s=float(self.o3_slope_off.value()),
            height_on_db=float(self.o3_h_on.value()),
            height_off_db=float(self.o3_h_off.value()),
            min_dur_ms=int(self.o3_min_dur.value()),
            refractory_ms=int(self.o3_refrac.value()),
        )
        self.set_pass_markers(opt3, self.opt3_start, self.opt3_end, y_series=x_det)

        self.status.setText(
            f"{self.t_ms.size:,} samples | passes: opt1={len(opt1)} opt2={len(opt2)} opt3={len(opt3)}"
        )


def main():
    app = QtWidgets.QApplication(sys.argv)

    # Sometimes faster for lots of line segments; if it causes issues, comment out.
    try:
        pg.setConfigOptions(useOpenGL=True)
    except Exception:
        pass

    w = MainWindow(CSV_PATH)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
