# main.py
# Offline viewer for STREAMING prominence-based peak detector (per-tag)
#
# Detector (per tag):
#  - EMA smoothing with configurable alpha
#  - Maintains a running "valley" while idle
#  - Starts tracking when EMA rises above (valley + PROMINENCE_DB)
#      * Start time is the valley time (this matches a prominence-style definition)
#  - Tracks peak = max EMA while tracking
#  - Ends pass when EMA drops by DROP_DB from peak OR timeout
#  - Optional min separation between passes
#
# Plot:
#  - Raw RSSI (gray)
#  - EMA (yellow)
#  - Predicted pass markers (red): start CIRCLE, peak DIAMOND, end SQUARE
#  - events.csv markers (stars): start=gold, peak=magenta, end=cyan

import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore


RAW_CSV_PATH = r"C:\gitwork\xc_split_tracker\data\raw-3x3 approaches-12-20-2025.csv"
EVENTS_CSV_PATH = r"e:\events.csv"

# -------------------------
# Detector constants
# -------------------------
PROMINENCE_DB = 10.0       # rise above valley to start pass tracking
DROP_DB = 10.0             # drop from peak to end
MAX_PASS_TIME_MS = 20_000
MIN_SEP_MS = 0            # set e.g. 300-800 if you want extra de-bounce between passes


def _round_away_from_zero(x: float) -> int:
    return int(x + 0.5) if x >= 0 else int(x - 0.5)


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


# =====================================================
# Per-tag streaming peak detector (Python port of the "new guy")
# =====================================================
@dataclass
class PeakPass1Py:
    # Config
    ema_alpha: float = 0.01
    prominence_db: float = 4.0
    drop_db: float = 3.0
    max_pass_ms: int = 20_000
    min_sep_ms: int = 0

    # EMA
    ema_init: bool = False
    ema: float = 0.0

    # State
    tracking: bool = False

    # Valley (idle baseline minimum)
    valley: float = 0.0
    valley_t_wall: int = 0
    valley_rel_ms: int = 0
    valley_raw_rssi: int = 0

    # Peak (while tracking)
    peak: float = 0.0
    peak_t_wall: int = 0
    peak_rel_ms: int = 0
    peak_raw_rssi: int = 0

    # Pass start (defined at valley)
    start_t_wall: int = 0
    start_rel_ms: int = 0
    start_raw_rssi: int = 0

    last_finish_t_wall: int = 0

    # Output latch
    pass_ready: bool = False
    out_start_rel_ms: int = 0
    out_peak_rel_ms: int = 0
    out_end_rel_ms: int = 0
    out_start_rssi: int = 0
    out_peak_rssi: int = 0
    out_end_rssi: int = 0

    def process_sample(self, *, rel_ms: int, now_wall_ms: int, raw_rssi: float | int) -> bool:
        """
        Feed one sample.
        Returns True iff a pass finished on this sample (outputs latched).
        """
        self.pass_ready = False

        x = float(raw_rssi)

        # EMA update
        if not self.ema_init:
            self.ema_init = True
            self.ema = x

            # Init valley at first sample
            self.valley = self.ema
            self.valley_t_wall = int(now_wall_ms)
            self.valley_rel_ms = int(rel_ms)
            self.valley_raw_rssi = int(raw_rssi)
            return False

        a = float(self.ema_alpha)
        self.ema = a * x + (1.0 - a) * self.ema
        e = float(self.ema)

        # Optional min separation block (still keep valley fresh)
        if (not self.tracking) and self.min_sep_ms and self.last_finish_t_wall:
            if (int(now_wall_ms) - int(self.last_finish_t_wall)) < int(self.min_sep_ms):
                if e < self.valley:
                    self.valley = e
                    self.valley_t_wall = int(now_wall_ms)
                    self.valley_rel_ms = int(rel_ms)
                    self.valley_raw_rssi = int(raw_rssi)
                return False

        # Idle: update valley and start when rise >= prominence
        if not self.tracking:
            if e < self.valley:
                self.valley = e
                self.valley_t_wall = int(now_wall_ms)
                self.valley_rel_ms = int(rel_ms)
                self.valley_raw_rssi = int(raw_rssi)

            if e >= (self.valley + float(self.prominence_db)):
                self.tracking = True

                # Start defined at valley
                self.start_t_wall = int(self.valley_t_wall)
                self.start_rel_ms = int(self.valley_rel_ms)
                self.start_raw_rssi = int(self.valley_raw_rssi)

                # Init peak at current sample
                self.peak = e
                self.peak_t_wall = int(now_wall_ms)
                self.peak_rel_ms = int(rel_ms)
                self.peak_raw_rssi = int(raw_rssi)

            return False

        # Tracking: update peak
        if e > self.peak:
            self.peak = e
            self.peak_t_wall = int(now_wall_ms)
            self.peak_rel_ms = int(rel_ms)
            self.peak_raw_rssi = int(raw_rssi)

        # Finish: drop-from-peak OR timeout
        dropped = e <= (self.peak - float(self.drop_db))
        timed_out = (self.max_pass_ms > 0) and ((int(now_wall_ms) - int(self.start_t_wall)) >= int(self.max_pass_ms))

        if dropped or timed_out:
            self.tracking = False
            self.last_finish_t_wall = int(now_wall_ms)

            self.pass_ready = True
            self.out_start_rel_ms = int(self.start_rel_ms)
            self.out_peak_rel_ms = int(self.peak_rel_ms)
            self.out_end_rel_ms = int(rel_ms)

            self.out_start_rssi = int(self.start_raw_rssi)
            self.out_peak_rssi = _round_away_from_zero(self.peak)  # match your earlier "Peak = rounded EMA"
            self.out_end_rssi = int(raw_rssi)

            # Reset valley from current point for next cycle
            self.valley = e
            self.valley_t_wall = int(now_wall_ms)
            self.valley_rel_ms = int(rel_ms)
            self.valley_raw_rssi = int(raw_rssi)

            return True

        return False

    def pop_pass(self) -> Optional[Tuple[int, int, int, int, int, int]]:
        """
        Returns (start_rel_ms, peak_rel_ms, end_rel_ms, start_rssi, peak_rssi, end_rssi) if ready.
        """
        if not self.pass_ready:
            return None
        self.pass_ready = False
        return (
            int(self.out_start_rel_ms),
            int(self.out_peak_rel_ms),
            int(self.out_end_rel_ms),
            int(self.out_start_rssi),
            int(self.out_peak_rssi),
            int(self.out_end_rssi),
        )


class PassManagerPy:
    """
    Owns per-tag PeakPass1Py objects. Top-level: lookup tag -> delegate.
    """

    def __init__(self, *, prominence_db: float, drop_db: float, max_pass_ms: int, min_sep_ms: int):
        self.prominence_db = float(prominence_db)
        self.drop_db = float(drop_db)
        self.max_pass_ms = int(max_pass_ms)
        self.min_sep_ms = int(min_sep_ms)
        self.tags: Dict[int, PeakPass1Py] = {}

    def _get(self, tag_id: int, ema_alpha: float) -> PeakPass1Py:
        tid = int(tag_id)
        det = self.tags.get(tid)
        if det is None:
            det = PeakPass1Py(
                ema_alpha=float(ema_alpha),
                prominence_db=self.prominence_db,
                drop_db=self.drop_db,
                max_pass_ms=self.max_pass_ms,
                min_sep_ms=self.min_sep_ms,
            )
            self.tags[tid] = det
        else:
            # Keep alpha in sync with UI
            det.ema_alpha = float(ema_alpha)
        return det

    def run_series(
        self,
        t_ms: np.ndarray,
        rssi_raw: np.ndarray,
        tag_ids: np.ndarray,
        *,
        ema_alpha: float,
    ) -> Tuple[List[Tuple[int, int, int]], np.ndarray]:
        """
        Returns:
          triplets: list[(start_i, peak_i, end_i)]  indices into the arrays
          ema_series: per-sample EMA (per-tag detector EMA) float32
        """
        n = int(t_ms.size)
        ema_series = np.empty(n, dtype=np.float32)
        triplets: List[Tuple[int, int, int]] = []

        # For converting rel_ms back to indices quickly for this filtered view:
        # rel_ms may repeat, so map to last index; that’s fine for plotting markers.
        rel_to_i: Dict[int, int] = {}
        for i in range(n):
            rel_to_i[int(t_ms[i])] = i

        for i in range(n):
            ti = int(t_ms[i])
            ri = float(rssi_raw[i])
            tag = int(tag_ids[i])

            det = self._get(tag, ema_alpha=ema_alpha)
            finished = det.process_sample(rel_ms=ti, now_wall_ms=ti, raw_rssi=ri)
            ema_series[i] = float(det.ema)

            if finished:
                out = det.pop_pass()
                if out is None:
                    continue
                s_rel, p_rel, e_rel, *_ = out

                s_i = rel_to_i.get(int(s_rel), None)
                p_i = rel_to_i.get(int(p_rel), None)
                e_i = rel_to_i.get(int(e_rel), None)

                if s_i is not None and p_i is not None and e_i is not None:
                    triplets.append((int(s_i), int(p_i), int(e_i)))

        return triplets, ema_series


# -------------------------
# UI
# -------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, raw_path: str, events_path: str):
        super().__init__()
        self.setWindowTitle("RSSI Plot — per-tag streaming peak detector (prominence/drop)")
        self.resize(2000, 980)

        self.df_raw = load_raw_csv(raw_path)
        self.df_evt = load_events_csv_or_empty(events_path)

        self.t_ms = np.array([], dtype=np.int64)
        self.rssi = np.array([], dtype=np.float32)
        self.tag_ids = np.array([], dtype=np.int64)

        self.raw_header = "unix_ms,rel_ms,tag_id,pass_id,rssi"
        self.raw_row_offset = 1

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

        # Params
        r = 1
        controls.addWidget(QtWidgets.QLabel("Params:"), r, 0)

        self.ema_alpha = QtWidgets.QDoubleSpinBox()
        self.ema_alpha.setDecimals(6)
        self.ema_alpha.setRange(0.0, 1.0)
        self.ema_alpha.setSingleStep(0.001)
        self.ema_alpha.setValue(0.01)
        self.ema_alpha.setFixedWidth(90)

        self.prominence = QtWidgets.QDoubleSpinBox()
        self.prominence.setDecimals(2)
        self.prominence.setRange(0.0, 50.0)
        self.prominence.setSingleStep(0.25)
        self.prominence.setValue(PROMINENCE_DB)
        self.prominence.setFixedWidth(90)

        self.drop = QtWidgets.QDoubleSpinBox()
        self.drop.setDecimals(2)
        self.drop.setRange(0.0, 50.0)
        self.drop.setSingleStep(0.25)
        self.drop.setValue(DROP_DB)
        self.drop.setFixedWidth(90)

        c = 1
        controls.addWidget(QtWidgets.QLabel("ema α"), r, c); c += 1
        controls.addWidget(self.ema_alpha, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("prom dB"), r, c); c += 1
        controls.addWidget(self.prominence, r, c); c += 1
        controls.addWidget(QtWidgets.QLabel("drop dB"), r, c); c += 1
        controls.addWidget(self.drop, r, c)

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

        # EMA line
        self.ema_line = pg.PlotDataItem(pen=pg.mkPen((255, 180, 0, 220), width=2))
        self.ema_line.setDownsampling(auto=True, method="peak")
        self.ema_line.setClipToView(True)
        self.plot.addItem(self.ema_line)

        # Predicted markers (red): start circle, peak diamond, end square
        self.calc_start = pg.ScatterPlotItem(symbol="o", size=10,
                                             pen=pg.mkPen((255, 0, 0, 230)),
                                             brush=pg.mkBrush((255, 0, 0, 160)))
        self.calc_peak = pg.ScatterPlotItem(symbol="d", size=10,
                                            pen=pg.mkPen((255, 0, 0, 230)),
                                            brush=pg.mkBrush((255, 0, 0, 160)))
        self.calc_end = pg.ScatterPlotItem(symbol="s", size=10,
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

        self.ema_alpha.valueChanged.connect(self.recompute)
        self.prominence.valueChanged.connect(self.recompute)
        self.drop.valueChanged.connect(self.recompute)

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
        idx = points[0].data()
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
            self.tag_ids = np.array([], dtype=np.int64)
            self.raw_scatter.setData([], [])
            self.raw_line.setData([], [])
            self.ema_line.setData([], [])
            self.raw_list.clear()
            self.calc_start.setData([], [])
            self.calc_peak.setData([], [])
            self.calc_end.setData([], [])
            self.evt_start.setData([], [])
            self.evt_peak.setData([], [])
            self.evt_end.setData([], [])
            return

        self.t_ms = df["rel_ms"].to_numpy(dtype=np.int64)
        self.rssi = df["rssi"].to_numpy(dtype=np.float32)
        self.tag_ids = df["tag_id"].to_numpy(dtype=np.int64)

        self.raw_line.setData(self.t_ms, self.rssi)
        idx = np.arange(len(self.t_ms), dtype=np.int32)
        self.raw_scatter.setData(x=self.t_ms, y=self.rssi, data=idx)

        # Raw list
        self.raw_list.clear()
        self.raw_list.addItem(self.raw_header)

        has_unix = "unix_ms" in df.columns
        unix_vals = df["unix_ms"].to_numpy(dtype="int64", na_value=-1) if has_unix else None

        tag_vals = df["tag_id"].to_numpy(dtype=np.int64)
        pass_vals = df["pass_id"].to_numpy(dtype=np.int64)

        for i in range(len(self.t_ms)):
            u_str = ""
            if has_unix:
                u = unix_vals[i]
                u_str = "" if u == -1 else str(int(u))
            line = f"{u_str},{int(self.t_ms[i])},{int(tag_vals[i])},{int(pass_vals[i])},{int(round(float(self.rssi[i])))}"
            self.raw_list.addItem(line)

        # ticks
        x0 = float(self.t_ms[0])
        x1 = float(self.t_ms[-1])

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
    def set_triplet_markers(self, triplets, ema_series: np.ndarray):
        if not triplets:
            self.calc_start.setData([], [])
            self.calc_peak.setData([], [])
            self.calc_end.setData([], [])
            return

        xs_s, ys_s = [], []
        xs_p, ys_p = [], []
        xs_e, ys_e = [], []
        n = int(self.t_ms.size)

        for s, p, e in triplets:
            s = int(s); p = int(p); e = int(e)
            if 0 <= s < n:
                xs_s.append(float(self.t_ms[s]))
                ys_s.append(float(ema_series[s]))
            if 0 <= p < n:
                xs_p.append(float(self.t_ms[p]))
                ys_p.append(float(ema_series[p]))
            if 0 <= e < n:
                xs_e.append(float(self.t_ms[e]))
                ys_e.append(float(ema_series[e]))

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

        self.evt_start.setData(t[et == 1].astype(np.float64), y[et == 1].astype(np.float64))
        self.evt_peak.setData(t[et == 2].astype(np.float64), y[et == 2].astype(np.float64))
        self.evt_end.setData(t[et == 3].astype(np.float64), y[et == 3].astype(np.float64))

    # -------------------------
    # Recompute overlays
    # -------------------------
    def recompute(self):
        if self.t_ms is None or self.t_ms.size == 0:
            return

        ema_alpha = float(self.ema_alpha.value())
        prom = float(self.prominence.value())
        drop = float(self.drop.value())

        mgr = PassManagerPy(
            prominence_db=prom,
            drop_db=drop,
            max_pass_ms=MAX_PASS_TIME_MS,
            min_sep_ms=MIN_SEP_MS,
        )
        triplets, ema_series = mgr.run_series(self.t_ms, self.rssi, self.tag_ids, ema_alpha=ema_alpha)

        self.ema_line.setData(self.t_ms, ema_series)
        self.set_triplet_markers(triplets, ema_series)

        # events.csv stars filtered to selection
        df_evt = self.df_evt
        tag_text = self.tag_combo.currentText()
        pass_text = self.pass_combo.currentText()

        if tag_text != "(all)":
            df_evt = df_evt[df_evt["tag_id"] == int(tag_text)]
        if pass_text != "(all)":
            df_evt = df_evt[df_evt["pass_id"] == int(pass_text)]

        if self.t_ms.size and not df_evt.empty:
            x0 = int(self.t_ms[0])
            x1 = int(self.t_ms[-1])
            df_evt = df_evt[(df_evt["rel_ms"] >= x0) & (df_evt["rel_ms"] <= x1)]

        self.set_event_star_markers(df_evt)

        self.status.setText(
            f"{self.t_ms.size:,} samples | ema α={ema_alpha:g} | prom={prom:g}dB drop={drop:g}dB | "
            f"maxPass={MAX_PASS_TIME_MS}ms sep={MIN_SEP_MS}ms | passes={len(triplets)} | "
            f"events={0 if df_evt is None else len(df_evt)}"
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
