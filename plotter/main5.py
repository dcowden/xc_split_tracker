"""RSSI peak-drop session viewer matching the ESP32 session detector.

Replay uses rel_ms as the wall clock; UART buffering cannot be reconstructed.
"""
import argparse
import sys
from dataclasses import dataclass, field

import numpy as np
import pandas as pd
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from main3 import RAW_CSV_PATH, EVENTS_CSV_PATH, load_raw_csv, load_events_csv_or_empty

RESET_WINDOW_MS = 5000
EMA_ALPHA = 0.05
DROP_DB = 4.0
DROP_HOLD_MS = 2000
MAX_SESSION_MS = 35000
REARM_RISE_DB = 4.0


@dataclass
class SessionDetector:
    active: bool = False
    filtered_peak: bool = False
    peak_rssi: float = -127
    peak_rel_ms: int = 0
    last_rssi: int = -127
    last_rel_ms: int = 0
    last_wall_ms: int = 0
    readings: list = field(default_factory=list)
    times: list = field(default_factory=list)
    ema_alpha: float = EMA_ALPHA
    ema: float | None = None
    waiting: bool = False
    valley: float = -127
    start_ms: int = 0
    start_rssi: int = -127
    drop_since: int | None = None
    last_signal: float = -127

    def filter_sample(self, rel_ms, rssi):
        if self.readings and rel_ms - self.last_rel_ms >= 1000:
            self.readings.clear()
            self.times.clear()
            self.ema = None
            self.drop_since = None
        self.last_rssi = rssi
        self.last_rel_ms = rel_ms
        self.last_wall_ms = rel_ms
        self.readings = (self.readings + [rssi])[-3:]
        self.times = (self.times + [rel_ms])[-3:]
        if len(self.readings) == 3:
            median = sorted(self.readings)[1]
            self.ema = (median if self.ema is None else
                        self.ema_alpha * median + (1.0 - self.ema_alpha) * self.ema)
            self.last_signal = self.ema
            return self.ema, self.times[1], True
        self.last_signal = float(rssi)
        return float(rssi), rel_ms, False

    def begin(self, rel_ms, rssi, value, peak_time, filtered):
        self.active = True
        self.waiting = False
        self.start_ms = min(rel_ms, peak_time)
        self.start_rssi = rssi
        self.peak_rssi = value
        self.peak_rel_ms = peak_time
        self.filtered_peak = filtered
        self.drop_since = None


@dataclass
class Session:
    tag_id: int
    start_ms: int
    start_rssi: int
    peak_ms: int
    peak_rssi: float
    end_ms: int
    end_rssi: int
    confirmed_ms: int
    filtered: bool
    trailing: bool
    reason: str


def detect_sessions(df, reset_ms=RESET_WINDOW_MS, finish_trailing=True,
                    ema_alpha=EMA_ALPHA, drop_db=DROP_DB, drop_hold_ms=DROP_HOLD_MS,
                    max_session_ms=MAX_SESSION_MS, rearm_db=REARM_RISE_DB,
                    filtered_trace=None):
    """Replay all rows before display filtering, including independent idle timers."""
    if reset_ms <= 0:
        raise ValueError("Reset window must be positive")
    if not 0 < ema_alpha <= 1:
        raise ValueError("EMA alpha must be greater than zero and at most one")
    if drop_db <= 0 or rearm_db <= 0 or drop_hold_ms < 0 or max_session_ms <= 0:
        raise ValueError("Drop, rearm rise and max duration must be positive; hold cannot be negative")
    states, sessions = {}, []

    def finish(tag, confirmed, reason, trailing=False):
        det = states[tag]
        sessions.append(Session(
            tag, det.start_ms, det.start_rssi, det.peak_rel_ms, det.peak_rssi,
            det.last_rel_ms, det.last_rssi, confirmed,
            det.filtered_peak, trailing, reason,
        ))
        det.active = False
        det.waiting = True
        det.valley = det.last_signal
        det.drop_since = None

    def advance(now, trailing=False):
        for tag, det in list(states.items()):
            silence_at = det.last_rel_ms + reset_ms
            if det.active:
                maximum_at = det.start_ms + max_session_ms
                deadline = min(silence_at, maximum_at)
                if now >= deadline:
                    finish(tag, deadline, "silence" if silence_at <= maximum_at else "max duration", trailing)
            if now >= silence_at:
                # After true silence, the next packet is another first sighting.
                states.pop(tag)

    last_time = None
    for rel, tag, rssi in df[["rel_ms", "tag_id", "rssi"]].itertuples(index=False, name=None):
        rel, tag, rssi = int(rel), int(tag), int(rssi)
        last_time = rel
        advance(rel)
        det = states.setdefault(tag, SessionDetector(ema_alpha=ema_alpha))
        value, peak_time, filtered = det.filter_sample(rel, rssi)
        if filtered_trace is not None:
            filtered_trace.append((tag, peak_time, value if filtered else float("nan")))
        if not det.active:
            if det.waiting:
                # Rearm only on a fresh filtered rise out of the post-pass tail.
                # Raw warmup values after a gap do not rearm a waiting detector.
                if not filtered:
                    continue
                det.valley = min(det.valley, value)
                if value < det.valley + rearm_db:
                    continue
            # First packet starts immediately; subsequent passes require rearming.
            det.begin(rel, rssi, value, peak_time, filtered)
            continue
        if filtered or not det.filtered_peak:
            if (filtered and not det.filtered_peak) or value > det.peak_rssi:
                det.peak_rssi = value
                det.peak_rel_ms = peak_time
            det.filtered_peak = det.filtered_peak or filtered
        if filtered and value <= det.peak_rssi - drop_db:
            if det.drop_since is None:
                det.drop_since = rel
            # Require received observations across the hold; silence is separate.
            if rel - det.drop_since >= drop_hold_ms:
                finish(tag, rel, "drop")
        else:
            det.drop_since = None

    pending = sum(det.active for det in states.values())
    if finish_trailing and last_time is not None:
        # Explicit offline assumption: the file is followed by silence.
        advance(last_time + reset_ms, trailing=True)
        pending = 0
    sessions.sort(key=lambda s: (s.confirmed_ms, s.tag_id))
    return sessions, pending


class FrameModel(QtCore.QAbstractTableModel):
    def __init__(self, frame, parent=None):
        super().__init__(parent)
        self.frame = frame

    def rowCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(self.frame)

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(self.frame.columns)

    def data(self, index, role=QtCore.Qt.ItemDataRole.DisplayRole):
        if index.isValid() and role == QtCore.Qt.ItemDataRole.DisplayRole:
            value = self.frame.iat[index.row(), index.column()]
            return "" if pd.isna(value) else str(value)

    def headerData(self, section, orientation, role=QtCore.Qt.ItemDataRole.DisplayRole):
        if role == QtCore.Qt.ItemDataRole.DisplayRole:
            return str(self.frame.columns[section]) if orientation == QtCore.Qt.Orientation.Horizontal else str(section + 1)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, raw_path=RAW_CSV_PATH, events_path=EVENTS_CSV_PATH):
        super().__init__()
        self.setWindowTitle("RSSI Plot - Peak Drop Sessions (main5)")
        self.resize(1500, 900)
        self.df_raw = load_raw_csv(raw_path)
        self.df_evt = load_events_csv_or_empty(events_path)
        self.sessions = []
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        files = QtWidgets.QGridLayout()
        self.raw_path = QtWidgets.QLineEdit(raw_path)
        self.events_path = QtWidgets.QLineEdit(events_path)
        for row, (label, edit) in enumerate((("Raw CSV", self.raw_path), ("Events CSV", self.events_path))):
            files.addWidget(QtWidgets.QLabel(label), row, 0)
            files.addWidget(edit, row, 1)
            button = QtWidgets.QToolButton()
            button.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_DirOpenIcon))
            button.setToolTip("Open " + label)
            button.clicked.connect(lambda checked=False, e=edit: self.browse(e))
            files.addWidget(button, row, 2)
            edit.editingFinished.connect(self.reload)
        root.addLayout(files)

        controls = QtWidgets.QHBoxLayout()
        self.tag_combo = QtWidgets.QComboBox()
        self.reset_window = QtWidgets.QDoubleSpinBox()
        self.reset_window.setRange(0.001, 3600)
        self.reset_window.setDecimals(3)
        self.reset_window.setSingleStep(0.5)
        self.reset_window.setSuffix(" s")
        self.reset_window.setValue(RESET_WINDOW_MS / 1000)
        self.drop_threshold = QtWidgets.QDoubleSpinBox()
        self.drop_threshold.setRange(0.1, 100)
        self.drop_threshold.setSingleStep(0.5)
        self.drop_threshold.setSuffix(" dB")
        self.drop_threshold.setValue(DROP_DB)
        self.max_session = QtWidgets.QDoubleSpinBox()
        self.max_session.setRange(0.1, 3600)
        self.max_session.setSingleStep(5)
        self.max_session.setSuffix(" s")
        self.max_session.setValue(MAX_SESSION_MS / 1000)
        self.drop_hold = QtWidgets.QDoubleSpinBox()
        self.drop_hold.setRange(0, 60)
        self.drop_hold.setSingleStep(0.25)
        self.drop_hold.setSuffix(" s")
        self.drop_hold.setValue(DROP_HOLD_MS / 1000)
        self.rearm_rise = QtWidgets.QDoubleSpinBox()
        self.rearm_rise.setRange(0.1, 100)
        self.rearm_rise.setSingleStep(0.5)
        self.rearm_rise.setSuffix(" dB")
        self.rearm_rise.setValue(REARM_RISE_DB)
        self.ema_alpha = QtWidgets.QDoubleSpinBox()
        self.ema_alpha.setDecimals(4)
        self.ema_alpha.setRange(0.0001, 1.0)
        self.ema_alpha.setSingleStep(0.01)
        self.ema_alpha.setValue(EMA_ALPHA)
        self.ema_alpha.setToolTip("EMA after the three-reading median. 1 adds no smoothing; smaller values smooth more.")
        for label, widget in (("Tag", self.tag_combo), ("EMA alpha", self.ema_alpha), ("Drop threshold", self.drop_threshold), ("Max session", self.max_session)):
            controls.addWidget(QtWidgets.QLabel(label))
            controls.addWidget(widget)
        self.drop_threshold.setToolTip("Finish after the filtered signal stays this far below its peak for the drop hold time.")
        self.max_session.setToolTip("Maximum elapsed time from session start; finishing does not immediately rearm.")
        self.reset_window.setToolTip("Close after this long without any packets; the next packet starts immediately.")
        self.rearm_rise.setToolTip("After finishing, require this rise above the tail minimum before opening another session. First sighting has no rise requirement.")
        self.drop_hold.setToolTip("Received below-threshold observations must span this duration. Gaps of one second reset the hold.")
        controls.addStretch()
        root.addLayout(controls)
        secondary = QtWidgets.QHBoxLayout()
        for label, widget in (("Drop hold", self.drop_hold), ("Silence timeout", self.reset_window), ("Rearm rise", self.rearm_rise)):
            secondary.addWidget(QtWidgets.QLabel(label))
            secondary.addWidget(widget)
        secondary.addStretch()
        root.addLayout(secondary)

        overlays = QtWidgets.QHBoxLayout()
        self.show_predicted = QtWidgets.QCheckBox("Detected sessions")
        self.show_actual = QtWidgets.QCheckBox("Recorded events")
        self.show_filtered = QtWidgets.QCheckBox("Filtered signal")
        self.finish_trailing = QtWidgets.QCheckBox("Assume silence after EOF")
        self.finish_trailing.setToolTip("Advance the replay clock past the file's end to finish trailing sessions. Uncheck to leave them pending.")
        for box in (self.show_predicted, self.show_actual, self.show_filtered, self.finish_trailing):
            box.setChecked(True)
            overlays.addWidget(box)
        overlays.addStretch()
        root.addLayout(overlays)

        splitter = QtWidgets.QSplitter()
        root.addWidget(splitter, 1)
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "rel_ms", units="ms")
        self.plot.setLabel("left", "RSSI", units="dBm")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend()
        splitter.addWidget(self.plot)
        tabs = QtWidgets.QTabWidget()
        self.raw_table = QtWidgets.QTableView()
        self.session_table = QtWidgets.QTableView()
        for title, table in (("Sessions", self.session_table), ("Raw data", self.raw_table)):
            table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
            table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
            table.setAlternatingRowColors(True)
            tabs.addTab(table, title)
        splitter.addWidget(tabs)
        splitter.setSizes([1000, 500])
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        self.refresh_tags()
        self.tag_combo.currentIndexChanged.connect(self.recompute)
        self.reset_window.valueChanged.connect(self.recompute)
        for control in (self.drop_threshold, self.max_session, self.drop_hold, self.rearm_rise):
            control.valueChanged.connect(self.recompute)
        self.ema_alpha.valueChanged.connect(self.recompute)
        for box in (self.show_predicted, self.show_actual, self.show_filtered, self.finish_trailing):
            box.toggled.connect(self.recompute)
        self.recompute()

    def refresh_tags(self):
        old = self.tag_combo.currentText()
        self.tag_combo.blockSignals(True)
        self.tag_combo.clear()
        self.tag_combo.addItem("(all)")
        self.tag_combo.addItems([str(t) for t in sorted(self.df_raw.tag_id.unique())])
        index = self.tag_combo.findText(old)
        self.tag_combo.setCurrentIndex(max(0, index))
        self.tag_combo.blockSignals(False)

    def browse(self, edit):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open CSV", edit.text(), "CSV files (*.csv)")
        if path:
            edit.setText(path)
            self.reload()

    def reload(self):
        try:
            raw = load_raw_csv(self.raw_path.text())
            events = load_events_csv_or_empty(self.events_path.text())
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "CSV load failed", str(exc))
            return
        self.df_raw, self.df_evt = raw, events
        self.refresh_tags()
        self.recompute()

    def select_raw(self, item, points):
        if points:
            row = int(points[0].data())
            self.raw_table.selectRow(row)
            self.raw_table.scrollTo(self.raw_table.model().index(row, 0))

    @staticmethod
    def set_table_frame(table, frame):
        previous = table.model()
        table.setModel(FrameModel(frame, table))
        if previous is not None:
            previous.deleteLater()

    def recompute(self):
        reset_ms = round(self.reset_window.value() * 1000)
        alpha = self.ema_alpha.value()
        drop_db = self.drop_threshold.value()
        max_ms = round(self.max_session.value() * 1000)
        trace = []
        sessions, pending = detect_sessions(self.df_raw, reset_ms=reset_ms,
            finish_trailing=self.finish_trailing.isChecked(), ema_alpha=alpha,
            drop_db=drop_db, max_session_ms=max_ms,
            drop_hold_ms=round(self.drop_hold.value() * 1000),
            rearm_db=self.rearm_rise.value(), filtered_trace=trace)
        selected = self.tag_combo.currentText()
        df, events = self.df_raw, self.df_evt
        if selected != "(all)":
            tag = int(selected)
            df = df[df.tag_id == tag]
            events = events[events.tag_id == tag]
            sessions = [s for s in sessions if s.tag_id == tag]
        self.sessions = sessions
        self.plot.clear()
        for i, (tag, group) in enumerate(df.groupby("tag_id", sort=False)):
            self.plot.plot(group.rel_ms.to_numpy(), group.rssi.to_numpy(),
                           pen=pg.mkPen(pg.intColor(i, hues=max(3, df.tag_id.nunique())), width=1), name=f"Tag {tag}")
        raw = pg.ScatterPlotItem(x=df.rel_ms.to_numpy(), y=df.rssi.to_numpy(),
                                data=np.arange(len(df)), size=4, pen=None, brush="#82909b")
        raw.sigClicked.connect(self.select_raw)
        self.plot.addItem(raw)
        if self.show_filtered.isChecked():
            filtered = pd.DataFrame(trace, columns=["tag_id", "rel_ms", "rssi"])
            if selected != "(all)":
                filtered = filtered[filtered.tag_id == int(selected)]
            for tag, group in filtered.groupby("tag_id", sort=False):
                self.plot.plot(group.rel_ms.to_numpy(), group.rssi.to_numpy(),
                    connect="finite", pen=pg.mkPen("#ffcf66", width=2), name=f"Filtered {tag}")
        if self.show_predicted.isChecked():
            for name, symbol, color, time_field, value_field in (
                ("Start", "o", "#51c78b", "start_ms", "start_rssi"),
                ("Peak", "d", "#ff5757", "peak_ms", "peak_rssi"),
                ("End", "s", "#51bfe0", "end_ms", "end_rssi"),
            ):
                self.plot.addItem(pg.ScatterPlotItem(
                    x=[getattr(s, time_field) for s in sessions],
                    y=[getattr(s, value_field) for s in sessions],
                    symbol=symbol, size=12, brush=color, pen="w", name=name))
        if self.show_actual.isChecked():
            for kind, color in ((1, "#ffd700"), (2, "#ff55dd"), (3, "#00ffff")):
                rows = events[events.event_type == kind]
                self.plot.addItem(pg.ScatterPlotItem(x=rows.rel_ms.to_numpy(), y=rows.rssi.to_numpy(),
                    symbol="star", size=15, brush=color, pen=None, name=f"Recorded {kind}"))
        self.set_table_frame(self.raw_table, df.reset_index(drop=True))
        records = [{"tag_id": s.tag_id, "start_ms": s.start_ms, "peak_ms": s.peak_ms,
                    "peak_dBm": round(s.peak_rssi, 3), "end_ms": s.end_ms,
                    "confirmed_ms": s.confirmed_ms,
                    "duration_s": round((s.confirmed_ms - s.start_ms) / 1000, 3),
                    "end_reason": s.reason,
                    "peak_method": ("median + EMA" if alpha < 1 else "median") if s.filtered else "raw fallback",
                    "after_EOF": s.trailing} for s in sessions]
        columns = ["tag_id", "start_ms", "peak_ms", "peak_dBm", "end_reason", "duration_s", "end_ms", "confirmed_ms", "peak_method", "after_EOF"]
        self.set_table_frame(self.session_table, pd.DataFrame(records, columns=columns))
        trailing = sum(s.trailing for s in sessions)
        self.status.setText(f"{len(df):,} samples | {len(sessions)} peaks | {pending} pending (all tags) | "
                            f"{trailing} completed after EOF | drop {drop_db:g} dB | max {max_ms / 1000:g} s | alpha {alpha:g}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_csv", nargs="?", default=RAW_CSV_PATH)
    parser.add_argument("--events", default=EVENTS_CSV_PATH)
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv[:1])
    try:
        window = MainWindow(args.raw_csv, args.events)
    except Exception as exc:
        QtWidgets.QMessageBox.critical(None, "CSV load failed", str(exc))
        return 1
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
