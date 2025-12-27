# main.py
# Offline viewer that MATCHES your current pass_processor.h behavior.
# - Detection is FLOOR-referenced + tent-guard arming + hold-time start + floor stop (PASS_STOP_MODE=0)
# - EMA uses g_cfg.ema_alpha (default 0.005)
# - Samples with raw rssi < FLOOR are ignored entirely (no EMA update, no arming, no candidate timing) — exactly like firmware.
#
# Plot:
# - Raw RSSI (gray)
# - Firmware EMA (yellow)
# - Predicted pass markers (red): start square, peak diamond, end circle
# - events.csv markers (stars): start=gold, peak=magenta, end=cyan
#
# Also: ignores missing events.csv cleanly.

import os
import sys
import numpy as np
import pandas as pd

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


#RAW_CSV_PATH = r"e:\raw.csv"
RAW_CSV_PATH = r"C:\gitwork\xc_split_tracker\data\raw-3x3 approaches-12-20-2025.csv"
EVENTS_CSV_PATH = r"e:\events.csv"


# -------------------------
# CSV load
# -------------------------
def load_raw_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, engine="c")
    df.columns = [c.strip() for c in df.columns]

    required = {"rel_ms", "rssi", "tag_id", "pass_id"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"RAW CSV missing required columns: {sorted(missing)}")

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


def load_events_csv_or_empty(path: str) -> pd.DataFrame:
    """
    If file is missing, return an empty df with expected columns.
    If file exists but is malformed, raise (so you notice).
    """
    if not path or not os.path.exists(path):
        return pd.DataFrame(columns=["rel_ms", "tag_id", "pass_id", "event_type", "rssi", "unix_ms"])

    df = pd.read_csv(path, engine="c")
    df.columns = [c.strip() for c in df.columns]

    required = {"rel_ms", "tag_id", "pass_id", "event_type", "rssi"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"EVENTS CSV missing required columns: {sorted(missing)}")

    for col in ["rel_ms", "tag_id", "pass_id", "event_type", "rssi"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=["rel_ms", "tag_id", "pass_id", "event_type", "rssi"])

    df["rel_ms"] = df["rel_ms"].astype(np.int64)
    df["tag_id"] = df["tag_id"].astype(np.int64)
    df["pass_id"] = df["pass_id"].astype(np.int64)
    df["event_type"] = df["event_type"].astype(np.int64)
    df["rssi"] = df["rssi"].astype(np.float32)

    if "unix_ms" in df.columns:
        df["unix_ms"] = pd.to_numeric(df["unix_ms"], errors="coerce").astype("Int64")

    df = df.sort_values("rel_ms", kind="mergesort")
    return df


# -------------------------
# Firmware-exact EMA series
# -------------------------
def firmware_ema_series(
    rssi: np.ndarray,
    floor_dbm: float,
    ema_alpha: float,
) -> np.ndarray:
    """
    Matches pass_process_sample():
      if rssi < floor: return (no EMA update)
      if first accepted: ema = rssi
      else ema = a*rssi + (1-a)*ema
    For plotting, we output NaN when EMA hasn't initialized yet,
    and we keep NaN for floor-rejected samples (later we can forward-fill for a smooth line).
    """
    n = rssi.size
    y = np.empty(n, dtype=np.float32)
    a = float(ema_alpha)

    ema = None
    for i in range(n):
        ri = float(rssi[i])
        if ri < float(floor_dbm):
            y[i] = np.nan if ema is None else np.nan  # keep NaN for rejected samples (we'll ffill for plot)
            continue

        if ema is None:
            ema = ri
        else:
            ema = a * ri + (1.0 - a) * float(ema)

        y[i] = float(ema)

    if ema is None:
        y[:] = np.nan
    return y


# -------------------------
# Firmware-matching pass detection
# -------------------------
def passes_firmware_floor_tent_guard(
    t_ms: np.ndarray,
    rssi_raw: np.ndarray,
    floor_dbm: float,
    ema_alpha: float,
    delta_on_db: float,
    delta_off_db: float,
    min_ms: int,
    refrac_ms: int,
    tent_low_n: int,
):
    """
    Emulates CURRENT pass_processor.h behavior:
      - ignores samples with raw rssi < floor (no state changes)
      - EMA over accepted samples only
      - tent guard: requires EMA <= (floor+delta_off) for N accepted samples to arm
      - start: armed AND EMA >= (floor+delta_on) continuously for min_ms (timed by rel_ms as wall proxy)
      - stop: EMA <= (floor+delta_off)  (PASS_STOP_MODE=0)
      - refractory: after end, block new start for refrac_ms
      - peak: max EMA within pass
    Returns:
      triplets: list[(start_i, peak_i, end_i)]
      ema_series: np.ndarray (same length; NaN for rejected/uninitialized samples)
    """
    n = t_ms.size
    if n == 0:
        return [], np.array([], dtype=np.float32)

    arm_thr = float(floor_dbm + delta_off_db)
    on_thr = float(floor_dbm + delta_on_db)
    stop_thr = arm_thr  # PASS_STOP_MODE=0

    ema_series = firmware_ema_series(rssi_raw, floor_dbm=floor_dbm, ema_alpha=ema_alpha)

    in_pass = False
    armed = False
    lowN = 0

    candidate_t0 = None
    last_end_t = None

    start_i = None
    peak_i = None
    peak_ema = None

    triplets = []

    # IMPORTANT: firmware timing uses now_wall_ms (millis), but rel_ms is close enough for offline.
    for i in range(n):
        ti = int(t_ms[i])
        ri = float(rssi_raw[i])

        # Firmware hard-gate: ignore sample completely
        if ri < float(floor_dbm):
            continue

        ei = float(ema_series[i])
        if np.isnan(ei):
            continue

        above_on = (ei >= on_thr)
        below_stop = (ei <= stop_thr)

        # --- Arming (only when NOT in pass and NOT armed) ---
        if (not in_pass) and (not armed):
            if ei <= arm_thr:
                lowN = min(255, lowN + 1)
                if lowN >= int(tent_low_n):
                    armed = True
            else:
                lowN = 0

        # --- Start logic ---
        if not in_pass:
            # Refractory
            if last_end_t is not None and (ti - int(last_end_t)) < int(refrac_ms):
                candidate_t0 = None
                continue

            if not armed:
                candidate_t0 = None
                continue

            if above_on:
                if candidate_t0 is None:
                    candidate_t0 = ti

                held = ti - int(candidate_t0)
                if held >= int(min_ms):
                    # START
                    in_pass = True
                    start_i = i
                    peak_i = i
                    peak_ema = ei

                    candidate_t0 = None
                    # firmware: after start, require re-arming again before another start
                    armed = False
                    lowN = 0
            else:
                candidate_t0 = None

            continue

        # --- In-pass peak tracking ---
        if peak_ema is None or ei > float(peak_ema):
            peak_ema = float(ei)
            peak_i = i

        # --- Stop ---
        if below_stop:
            end_i = i
            triplets.append((int(start_i), int(peak_i), int(end_i)))

            in_pass = False
            last_end_t = ti

            start_i = None
            peak_i = None
            peak_ema = None

            candidate_t0 = None
            # firmware: require low band again
            armed = False
            lowN = 0

    return triplets, ema_series


def forward_fill_nans(y: np.ndarray) -> np.ndarray:
    """For nicer plotting: forward-fill NaNs (keeps leading NaNs as NaN)."""
    if y.size == 0:
        return y
    out = y.copy()
    last = np.nan
    for i in range(out.size):
        if np.isnan(out[i]):
            out[i] = last
        else:
            last = out[i]
    return out


# -------------------------
# UI
# -------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, raw_path: str, events_path: str):
        super().__init__()
        self.setWindowTitle("RSSI Plot — firmware-matching pass detector (floor + tent guard + EMA)")
        self.resize(2000, 980)

        self.df_raw = load_raw_csv(raw_path)
        self.df_evt = load_events_csv_or_empty(events_path)

        self.t_ms = np.array([], dtype=np.int64)
        self.rssi = np.array([], dtype=np.float32)

        # For raw panel mapping
        self.raw_header = "unix_ms,rel_ms,tag_id,pass_id,rssi"
        self.raw_row_offset = 1  # header is row 0

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        controls = QtWidgets.QGridLayout()
        controls.setHorizontalSpacing(10)
        controls.setVerticalSpacing(6)

        # Filters
        self.tag_combo = QtWidgets.QComboBox()
        self.pass_combo = QtWidgets.QComboBox()

        self.tag_combo.addItem("(all)")
        for t in sorted(self.df_raw["tag_id"].unique().tolist()):
            self.tag_combo.addItem(str(t))

        self.pass_combo.addItem("(all)")
        for p in sorted(self.df_raw["pass_id"].unique().tolist()):
            self.pass_combo.addItem(str(p))

        controls.addWidget(QtWidgets.QLabel("tag_id:"), 0, 0)
        controls.addWidget(self.tag_combo, 0, 1)
        controls.addWidget(QtWidgets.QLabel("pass_id:"), 0, 2)
        controls.addWidget(self.pass_combo, 0, 3)

        self.status = QtWidgets.QLabel("")
        self.status.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
        controls.addWidget(self.status, 0, 9, 1, 3)

        # Firmware parameters (match pass_processor.h defaults)
        r = 1
        controls.addWidget(QtWidgets.QLabel("Firmware params:"), r, 0)

        self.fw_ema_alpha = QtWidgets.QDoubleSpinBox()
        self.fw_ema_alpha.setDecimals(6)
        self.fw_ema_alpha.setRange(0.0, 1.0)
        self.fw_ema_alpha.setSingleStep(0.001)
        self.fw_ema_alpha.setValue(0.005)  # g_cfg.ema_alpha
        self.fw_ema_alpha.setFixedWidth(90)

        self.fw_floor = QtWidgets.QDoubleSpinBox()
        self.fw_floor.setDecimals(1)
        self.fw_floor.setRange(-140.0, 0.0)
        self.fw_floor.setSingleStep(1.0)
        self.fw_floor.setValue(-100.0)  # PASS_RSSI_FLOOR_DBM
        self.fw_floor.setFixedWidth(80)

        self.fw_don = QtWidgets.QDoubleSpinBox()
        self.fw_don.setDecimals(2)
        self.fw_don.setRange(0.0, 50.0)
        self.fw_don.setSingleStep(0.5)
        self.fw_don.setValue(5.0)  # PASS_DELTA_ON_DB
        self.fw_don.setFixedWidth(70)

        self.fw_doff = QtWidgets.QDoubleSpinBox()
        self.fw_doff.setDecimals(2)
        self.fw_doff.setRange(-50.0, 50.0)
        self.fw_doff.setSingleStep(0.5)
        self.fw_doff.setValue(0.0)  # PASS_DELTA_OFF_DB
        self.fw_doff.setFixedWidth(70)

        self.fw_min_ms = QtWidgets.QSpinBox()
        self.fw_min_ms.setRange(0, 600000)
        self.fw_min_ms.setSingleStep(10)
        self.fw_min_ms.setValue(200)  # PASS_MIN_MS
        self.fw_min_ms.setFixedWidth(80)

        self.fw_refrac = QtWidgets.QSpinBox()
        self.fw_refrac.setRange(0, 600000)
        self.fw_refrac.setSingleStep(50)
        self.fw_refrac.setValue(5000)  # PASS_REFRAC_MS
        self.fw_refrac.setFixedWidth(90)

        self.fw_tentN = QtWidgets.QSpinBox()
        self.fw_tentN.setRange(0, 255)
        self.fw_tentN.setSingleStep(1)
        self.fw_tentN.setValue(3)  # PASS_TENT_REQUIRE_LOW_BAND_SAMPLES
        self.fw_tentN.setFixedWidth(60)

        c = 1
        controls.addWidget(QtWidgets.QLabel("ema α"), r, c); c += 1
        controls.addWidget(self.fw_ema_alpha, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("floor"), r, c); c += 1
        controls.addWidget(self.fw_floor, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("Δon"), r, c); c += 1
        controls.addWidget(self.fw_don, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("Δoff"), r, c); c += 1
        controls.addWidget(self.fw_doff, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("min ms"), r, c); c += 1
        controls.addWidget(self.fw_min_ms, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("refrac ms"), r, c); c += 1
        controls.addWidget(self.fw_refrac, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("tent N"), r, c); c += 1
        controls.addWidget(self.fw_tentN, r, c)

        # Apply button
        self.apply_btn = QtWidgets.QPushButton("Recompute")
        controls.addWidget(self.apply_btn, 2, 0, 1, 2)

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
        self.raw_list.setStyleSheet("""QListWidget::item:selected { background: #2b6cb0; color: white; }""")
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

        # Raw series
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

        # Firmware EMA (yellow)
        self.fw_ema_line = pg.PlotDataItem(pen=pg.mkPen((255, 180, 0, 220), width=2))
        self.fw_ema_line.setDownsampling(auto=True, method="peak")
        self.fw_ema_line.setClipToView(True)
        self.plot.addItem(self.fw_ema_line)

        # Predicted markers (red)
        self.calc_start = pg.ScatterPlotItem(symbol="s", size=10,
                                             pen=pg.mkPen((255, 0, 0, 230)),
                                             brush=pg.mkBrush((255, 0, 0, 160)))
        self.calc_peak = pg.ScatterPlotItem(symbol="d", size=10,
                                            pen=pg.mkPen((255, 0, 0, 230)),
                                            brush=pg.mkBrush((255, 0, 0, 160)))
        self.calc_end = pg.ScatterPlotItem(symbol="o", size=10,
                                           pen=pg.mkPen((255, 0, 0, 230)),
                                           brush=pg.mkBrush((255, 0, 0, 160)))
        self.plot.addItem(self.calc_start)
        self.plot.addItem(self.calc_peak)
        self.plot.addItem(self.calc_end)

        # events.csv markers (stars)
        self.evt_start = pg.ScatterPlotItem(symbol="star", size=12,
                                            pen=pg.mkPen((255, 215, 0, 240)),
                                            brush=pg.mkBrush((255, 215, 0, 120)))
        self.evt_peak = pg.ScatterPlotItem(symbol="star", size=12,
                                           pen=pg.mkPen((255, 0, 255, 240)),
                                           brush=pg.mkBrush((255, 0, 255, 110)))
        self.evt_end = pg.ScatterPlotItem(symbol="star", size=12,
                                          pen=pg.mkPen((0, 255, 255, 240)),
                                          brush=pg.mkBrush((0, 255, 255, 110)))
        self.plot.addItem(self.evt_start)
        self.plot.addItem(self.evt_peak)
        self.plot.addItem(self.evt_end)

        # Clicking raw points -> highlight row
        self.raw_scatter.sigClicked.connect(self.on_points_clicked)

        # Wiring
        self.apply_btn.clicked.connect(self.recompute)
        self.tag_combo.currentIndexChanged.connect(self.on_filter_changed)
        self.pass_combo.currentIndexChanged.connect(self.on_filter_changed)

        for w in [self.fw_ema_alpha, self.fw_floor, self.fw_don, self.fw_doff, self.fw_min_ms, self.fw_refrac, self.fw_tentN]:
            w.valueChanged.connect(self.recompute)

        # Initial
        self.apply_filters_and_plot_raw()
        self.recompute()

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
        if not points:
            return
        p0 = points[0]
        idx = p0.data()
        if idx is None:
            return
        self.highlight_row(int(idx))

    # -------------------------
    # Filtering + plotting
    # -------------------------
    def on_filter_changed(self):
        self.apply_filters_and_plot_raw()
        self.recompute()

    def apply_filters_and_plot_raw(self):
        df = self.df_raw

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
            self.fw_ema_line.setData([], [])
            self.raw_list.clear()
            self.calc_start.setData([], [])
            self.calc_peak.setData([], [])
            self.calc_end.setData([], [])
            self.evt_start.setData([], [])
            self.evt_peak.setData([], [])
            self.evt_end.setData([], [])
            return

        t_ms = df["rel_ms"].to_numpy(dtype=np.int64)
        rssi = df["rssi"].to_numpy(dtype=np.float32)
        self.t_ms = t_ms
        self.rssi = rssi

        self.raw_line.setData(t_ms, rssi)
        idx = np.arange(len(t_ms), dtype=np.int32)
        self.raw_scatter.setData(x=t_ms, y=rssi, data=idx)

        # Raw list
        self.raw_list.clear()
        self.raw_list.addItem(self.raw_header)

        has_unix = "unix_ms" in df.columns
        unix_vals = df["unix_ms"].to_numpy(dtype="int64", na_value=-1) if has_unix else None

        tag_vals = df["tag_id"].to_numpy(dtype=np.int64)
        pass_vals = df["pass_id"].to_numpy(dtype=np.int64)

        for i in range(len(t_ms)):
            u_str = ""
            if has_unix:
                u = unix_vals[i]
                u_str = "" if u == -1 else str(int(u))
            line = f"{u_str},{int(t_ms[i])},{int(tag_vals[i])},{int(pass_vals[i])},{int(round(float(rssi[i])))}"
            self.raw_list.addItem(line)

        # ticks
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

    # -------------------------
    # Markers
    # -------------------------
    def set_triplet_markers(self, triplets, y_series: np.ndarray):
        if not triplets:
            self.calc_start.setData([], [])
            self.calc_peak.setData([], [])
            self.calc_end.setData([], [])
            return

        xs_s, ys_s = [], []
        xs_p, ys_p = [], []
        xs_e, ys_e = [], []
        n = self.t_ms.size

        for s, p, e in triplets:
            s = int(s); p = int(p); e = int(e)
            if 0 <= s < n:
                xs_s.append(float(self.t_ms[s]))
                ys_s.append(float(y_series[s]) if not np.isnan(y_series[s]) else float(self.rssi[s]))
            if 0 <= p < n:
                xs_p.append(float(self.t_ms[p]))
                ys_p.append(float(y_series[p]) if not np.isnan(y_series[p]) else float(self.rssi[p]))
            if 0 <= e < n:
                xs_e.append(float(self.t_ms[e]))
                ys_e.append(float(y_series[e]) if not np.isnan(y_series[e]) else float(self.rssi[e]))

        self.calc_start.setData(xs_s, ys_s)
        self.calc_peak.setData(xs_p, ys_p)
        self.calc_end.setData(xs_e, ys_e)

    def set_event_star_markers(self, df_evt_filtered: pd.DataFrame):
        if df_evt_filtered is None or df_evt_filtered.empty:
            self.evt_start.setData([], [])
            self.evt_peak.setData([], [])
            self.evt_end.setData([], [])
            return

        t = df_evt_filtered["rel_ms"].to_numpy(dtype=np.int64)
        y = df_evt_filtered["rssi"].to_numpy(dtype=np.float32)
        et = df_evt_filtered["event_type"].to_numpy(dtype=np.int64)

        m1 = (et == 1)
        m2 = (et == 2)
        m3 = (et == 3)

        self.evt_start.setData(t[m1].astype(np.float64), y[m1].astype(np.float64))
        self.evt_peak.setData(t[m2].astype(np.float64), y[m2].astype(np.float64))
        self.evt_end.setData(t[m3].astype(np.float64), y[m3].astype(np.float64))

    # -------------------------
    # Recompute overlays
    # -------------------------
    def recompute(self):
        if self.t_ms is None or self.t_ms.size == 0:
            return

        floor_dbm = float(self.fw_floor.value())
        ema_alpha = float(self.fw_ema_alpha.value())
        delta_on = float(self.fw_don.value())
        delta_off = float(self.fw_doff.value())
        min_ms = int(self.fw_min_ms.value())
        refrac_ms = int(self.fw_refrac.value())
        tentN = int(self.fw_tentN.value())

        triplets, ema_series = passes_firmware_floor_tent_guard(
            self.t_ms,
            self.rssi,
            floor_dbm=floor_dbm,
            ema_alpha=ema_alpha,
            delta_on_db=delta_on,
            delta_off_db=delta_off,
            min_ms=min_ms,
            refrac_ms=refrac_ms,
            tent_low_n=tentN,
        )

        # Firmware EMA line (yellow): forward-fill for display
        if ema_series.size:
            y_plot = forward_fill_nans(ema_series)
            self.fw_ema_line.setData(self.t_ms, y_plot)
        else:
            self.fw_ema_line.setData([], [])

        # Red markers at firmware EMA values
        self.set_triplet_markers(triplets, y_series=ema_series)

        # events.csv stars filtered to selection
        df_evt = self.df_evt
        tag_text = self.tag_combo.currentText()
        pass_text = self.pass_combo.currentText()

        if tag_text != "(all)":
            df_evt = df_evt[df_evt["tag_id"] == int(tag_text)]
        if pass_text != "(all)":
            df_evt = df_evt[df_evt["pass_id"] == int(pass_text)]

        if self.t_ms.size:
            x0 = int(self.t_ms[0])
            x1 = int(self.t_ms[-1])
            if not df_evt.empty:
                df_evt = df_evt[(df_evt["rel_ms"] >= x0) & (df_evt["rel_ms"] <= x1)]

        self.set_event_star_markers(df_evt)

        self.status.setText(
            f"{self.t_ms.size:,} samples | fw ema α={ema_alpha:g} floor={floor_dbm:g} "
            f"Δon={delta_on:g} Δoff={delta_off:g} min={min_ms} refrac={refrac_ms} tentN={tentN} "
            f"| fw passes={len(triplets)} | events={0 if df_evt is None else len(df_evt)}"
        )


def main():
    app = QtWidgets.QApplication(sys.argv)
    try:
        pg.setConfigOptions(useOpenGL=True)
    except Exception:
        pass

    w = MainWindow(RAW_CSV_PATH, EVENTS_CSV_PATH)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
