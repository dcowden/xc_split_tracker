# app.py
# Streamlit RSSI plotter using rel_ms (NO marker downsampling)
# - X axis: rel_ms shifted so t=0 at first sample
# - Raw data: markers only (ALL points)
# - EMA overlays: presets (0.01, 0.05, 0.1) + optional custom alpha
# - Legend click toggles series
# - Scroll wheel zoom enabled

import numpy as np
import pandas as pd
import streamlit as st
import plotly.graph_objects as go

CSV_PATH_DEFAULT = r"c:\gitwork\xc_split_tracker\data\raw-3x3 approaches-12-20-2025.csv"
EMA_PRESETS = (0.01, 0.05, 0.10)


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


@st.cache_data(show_spinner=False)
def load_df(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, engine="c")
    df.columns = [c.strip() for c in df.columns]

    required = {"rel_ms", "rssi", "tag_id", "pass_id"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Missing required columns: {sorted(missing)}")

    df["rel_ms"] = pd.to_numeric(df["rel_ms"], errors="coerce")
    df["rssi"] = pd.to_numeric(df["rssi"], errors="coerce")
    df["tag_id"] = pd.to_numeric(df["tag_id"], errors="coerce")
    df["pass_id"] = pd.to_numeric(df["pass_id"], errors="coerce")

    df = df.dropna(subset=["rel_ms", "rssi", "tag_id", "pass_id"])

    df["rel_ms"] = df["rel_ms"].astype(np.int64)
    df["rssi"] = df["rssi"].astype(np.float32)
    df["tag_id"] = df["tag_id"].astype(np.int64)
    df["pass_id"] = df["pass_id"].astype(np.int64)

    return df


def parse_alpha(s: str):
    """
    Parse a custom alpha string. Returns float or None if empty.
    Accepts: "0.07", ".07", "7e-2", etc.
    """
    if s is None:
        return None
    s = s.strip()
    if s == "":
        return None
    try:
        a = float(s)
    except ValueError:
        return "error"
    if not (0.0 < a <= 1.0):
        return "range"
    return a


st.set_page_config(layout="wide")
st.title("RSSI vs Time (rel_ms) — Raw Markers + EMA Experiment")

with st.sidebar:
    st.header("Input")
    path = st.text_input("CSV path", CSV_PATH_DEFAULT)

    st.header("Display")
    marker_size = st.slider("Raw marker size (px)", 2, 10, 5)
    show_raw = st.checkbox("Show raw markers", True)
    show_ema = st.checkbox("Show EMA lines", True)

    st.header("EMA alpha")
    preset_selected = st.multiselect(
        "Presets",
        options=[f"{a:g}" for a in EMA_PRESETS],
        default=[f"{a:g}" for a in EMA_PRESETS],
        help="Pick any preset alphas to overlay.",
    )
    custom_alpha_str = st.text_input(
        "Custom alpha (optional)",
        value="",
        help="Type a value like 0.07 (must be > 0 and <= 1). Leave blank to disable.",
    )

try:
    df = load_df(path)
except Exception as e:
    st.error(f"Failed to load CSV: {e}")
    st.stop()

# Optional filtering
tags = sorted(df["tag_id"].unique().tolist())
passes = sorted(df["pass_id"].unique().tolist())

tag_sel = st.sidebar.selectbox("tag_id", ["(all)"] + tags)
pass_sel = st.sidebar.selectbox("pass_id", ["(all)"] + passes)

df_f = df
if tag_sel != "(all)":
    df_f = df_f[df_f["tag_id"] == int(tag_sel)]
if pass_sel != "(all)":
    df_f = df_f[df_f["pass_id"] == int(pass_sel)]

if df_f.empty:
    st.error("No rows left after filtering.")
    st.stop()

# Sort by rel_ms
df_f = df_f.sort_values("rel_ms", kind="mergesort")

t_ms = df_f["rel_ms"].to_numpy(dtype=np.int64)
rssi = df_f["rssi"].to_numpy(dtype=np.float32)

t0 = int(t_ms[0])
t_s = (t_ms - t0) / 1000.0

st.caption(f"Plotting {len(t_s):,} samples (no downsampling).")

# Build alpha list from presets + custom
alphas = []
for s in preset_selected:
    try:
        alphas.append(float(s))
    except ValueError:
        pass

custom_parsed = parse_alpha(custom_alpha_str)
if custom_parsed == "error":
    st.sidebar.error("Custom alpha is not a valid number (examples: 0.07, .07, 7e-2).")
elif custom_parsed == "range":
    st.sidebar.error("Custom alpha must be > 0 and <= 1.")
elif isinstance(custom_parsed, float):
    # Avoid duplicate if it matches a preset
    if all(abs(custom_parsed - a) > 1e-12 for a in alphas):
        alphas.append(custom_parsed)

alphas = sorted(set(alphas))

# Compute EMAs
emas = {}
if show_ema and alphas:
    for a in alphas:
        emas[a] = ema_np(rssi, a)

fig = go.Figure()

# Raw markers (ALL points)
if show_raw:
    fig.add_trace(
        go.Scattergl(
            x=t_s,
            y=rssi,
            mode="markers",
            name="raw (markers)",
            marker=dict(size=marker_size, color="rgba(110,130,150,0.35)"),
        )
    )

# EMA lines (use a small palette; plotly will also auto-color if we omit color)
palette = [
    "rgba(255,165,0,0.95)",   # orange
    "rgba(50,205,50,0.95)",   # green
    "rgba(220,20,60,0.95)",   # crimson
    "rgba(30,144,255,0.95)",  # dodgerblue
    "rgba(148,0,211,0.95)",   # darkviolet
    "rgba(0,206,209,0.95)",   # darkturquoise
]
for i, (a, y) in enumerate(emas.items()):
    fig.add_trace(
        go.Scattergl(
            x=t_s,
            y=y,
            mode="lines",
            name=f"EMA α={a:g}",
            line=dict(width=2, color=palette[i % len(palette)]),
        )
    )

fig.update_xaxes(
    title="Time since start (s) [from rel_ms]",
    tick0=0.0,
    dtick=1.0,
    minor=dict(tick0=0.0, dtick=0.1, showgrid=True),
    showgrid=True,
    zeroline=True,
)
fig.update_yaxes(title="RSSI (dBm)", showgrid=True, zeroline=True)

fig.update_layout(
    height=750,
    hovermode="x unified",
    legend=dict(
        title="Click legend to hide/show",
        itemclick="toggle",
        itemdoubleclick="toggleothers",
    ),
    margin=dict(l=60, r=20, t=40, b=60),
)

st.plotly_chart(
    fig,
    use_container_width=True,
    config={
        "scrollZoom": True,
        "displaylogo": False,
        "modeBarButtonsToAdd": ["zoomIn2d", "zoomOut2d", "resetScale2d"],
    },
)

st.info(
    "All raw samples are plotted as markers (no downsampling). "
    "Use the preset checkboxes or type a custom alpha. "
    "Click legend items to hide/show series."
)
