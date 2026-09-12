"""Run with plotter's Python environment and a compiled session_replay executable."""
import csv
import io
import math
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / "plotter"))
import main5
import pandas as pd


def compare(executable, df):
    rows = df[["rel_ms", "tag_id", "rssi"]]
    stream = "\n".join(f"{int(t)} {int(tag)} {int(rssi)}" for t, tag, rssi in rows.itertuples(index=False, name=None))
    result = subprocess.run([str(executable)], input=stream, text=True, capture_output=True, check=True)
    actual = list(csv.reader(io.StringIO(result.stdout)))
    actual.sort(key=lambda row: (int(row[7]), int(row[0])))
    expected, pending = main5.detect_sessions(df)
    assert pending == 0 and len(actual) == len(expected), (len(actual), len(expected))
    for row, s in zip(actual, expected):
        assert [int(row[i]) for i in (0, 1, 2, 3, 5, 6, 7, 8)] == [
            s.tag_id, s.start_ms, s.start_rssi, s.peak_ms, s.end_ms,
            s.end_rssi, s.confirmed_ms, int(s.filtered)], (row, s)
        assert math.isclose(float(row[4]), s.peak_rssi, abs_tol=1e-10), (row, s)
        assert row[9] == s.reason, (row, s)
    return len(expected)


if __name__ == "__main__":
    exe = Path(sys.argv[1]).resolve()
    assert (main5.EMA_ALPHA, main5.DROP_DB, main5.MAX_SESSION_MS,
            main5.DROP_HOLD_MS, main5.RESET_WINDOW_MS, main5.REARM_RISE_DB) == (
                0.05, 4, 35000, 2000, 5000, 4)
    count = compare(exe, main5.load_raw_csv(main5.RAW_CSV_PATH))
    print(f"Recorded dataset: {count} identical sessions")
    rng = random.Random(2026)
    rows = []
    for tag in range(1, 5):
        t = 0
        for i in range(8000):
            t += rng.choice([5, 50, 100, 500, 999, 1000, 4999, 5000, 7000])
            rows.append((t, tag, rng.randint(-105, -35)))
    df = pd.DataFrame(sorted(rows), columns=["rel_ms", "tag_id", "rssi"])
    count = compare(exe, df)
    print(f"32,000 multi-tag samples: {count} identical sessions")
