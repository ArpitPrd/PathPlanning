import numpy as np

# ==============================
# Energy model (percent per timestep)
# ==============================
E_MAX      = 100.0  # start battery (%)
E_STEADY   = 1.0    # always paid each timestep
E_MOVEMENT = 2.0    # additional cost if next step is a move
E_TURN     = 1.0    # additional cost if heading changes while moving

# ==============================
# Heading + battery update
# ==============================
def _heading(p_from, p_to):
    """Return grid step direction as a tuple (dr, dc); (0,0) means no move."""
    return (int(p_to[0] - p_from[0]), int(p_to[1] - p_from[1]))

def battery_next(E_prev, p_prev, p_curr, p_next,
                 Esteady=E_STEADY, Emove=E_MOVEMENT, Eturn=E_TURN):
    """
    Compute next battery percentage given previous/current/next grid positions.
    - Always subtract Esteady.
    - If the UAV will move (p_next != p_curr), subtract Emove.
    - If moving and heading changes (prev heading != next heading), subtract Eturn.
    """
    E = E_prev - Esteady
    moving_next = (p_next[0] != p_curr[0]) or (p_next[1] != p_curr[1])
    if moving_next:
        E -= Emove
        moved_before = (p_curr[0] != p_prev[0]) or (p_curr[1] != p_prev[1])
        if moved_before:
            h1 = _heading(p_prev, p_curr)
            h2 = _heading(p_curr, p_next)
            if h1 != (0, 0) and h2 != (0, 0) and h1 != h2:
                E -= Eturn
    return max(0.0, E)

# ==============================
# Timeline helpers
# ==============================
def build_timelines(uav_paths, sink, T, *, pad_mode="hold"):
    """
    Build per-UAV position timelines of length T+1 (t=0..T).
    pad_mode:
      - "hold": repeat last position if path shorter than T
      - "sink": return to sink for remaining steps
    Returns dict[int] -> list[(r,c)] length T+1.
    """
    sink_tuple = tuple(sink.tolist()) if hasattr(sink, "tolist") else tuple(sink)
    timelines = {}
    for n, steps in uav_paths.items():
        steps = (steps or [])[:T]
        if len(steps) < T:
            if pad_mode == "sink":
                steps = steps + [sink_tuple] * (T - len(steps))
            else:
                last = steps[-1] if steps else sink_tuple
                steps = steps + [last] * (T - len(steps))
        timelines[n] = [sink_tuple] + steps
    return timelines



def compute_battery_timeline(timeline, *, E0=100.0, battery_fn=battery_next):
    """Return battery % over timeline (length T+1)."""
    T = len(timeline) - 1
    E = np.zeros(T + 1, dtype=float)
    E[0] = E0
    for t in range(1, T + 1):
        p_prev = timeline[t-2] if t-1 >= 1 else timeline[0]
        p_curr = timeline[t-1]
        p_next = timeline[t]
        E[t] = battery_fn(E[t-1], p_prev, p_curr, p_next)
    return E

def compute_battery_for_paths(uav_paths, sink, T, *, E0=100.0, battery_fn=battery_next, pad_mode="hold"):
    """
    (Kept the SAME signature and SAME outputs)
    Returns:
      battery_traces: dict[int] -> np.ndarray length T+1
      position_traces: dict[int] -> list[(r,c)] length T+1
    """
    sink_tuple = tuple(sink.tolist()) if hasattr(sink, "tolist") else tuple(sink)
    battery_traces = {}
    position_traces = {}

    for n, steps in uav_paths.items():
        steps = (steps or [])[:T]
        if len(steps) < T:
            if pad_mode == "sink":
                steps = steps + [sink_tuple] * (T - len(steps))
            else:  # "hold"
                last = steps[-1] if steps else sink_tuple
                steps = steps + [last] * (T - len(steps))

        timeline = [sink_tuple] + steps  # length T+1
        position_traces[n] = timeline
        battery_traces[n] = compute_battery_timeline(timeline, E0=E0, battery_fn=battery_fn)

    return battery_traces, position_traces

# ---------------------------
# NEW: Motion classification (purely post-hoc)
# ---------------------------
def _heading(p_from, p_to):
    """Grid step (dr, dc); (0,0) means no move."""
    return (int(p_to[0] - p_from[0]), int(p_to[1] - p_from[1]))

def classify_motion_timeline(timeline):
    """
    For t=1..T, label each step as 'steady' / 'straight' / 'turn'.
    Returns list[str] length T.
    """
    T = len(timeline) - 1
    labels = []
    for t in range(1, T + 1):
        p_prev = timeline[t-1]
        p_curr = timeline[t]
        moved = (p_curr[0] != p_prev[0]) or (p_curr[1] != p_prev[1])
        if not moved:
            labels.append('steady')
            continue

        h_now = _heading(p_prev, p_curr)
        if t - 1 >= 1:
            p_prev2 = timeline[t-2]
            moved_prev = (p_prev[0] != p_prev2[0]) or (p_prev[1] != p_prev2[1])
            if moved_prev:
                h_prev = _heading(p_prev2, p_prev)
                labels.append('turn' if (h_prev != (0,0) and h_now != (0,0) and h_prev != h_now) else 'straight')
            else:
                labels.append('straight')  # first move after steady
        else:
            labels.append('straight')      # first step has no prior heading
    return labels

def classify_from_timelines(position_traces):
    """position_traces -> motion_labels dict[int]->list[str]."""
    return {n: classify_motion_timeline(tl) for n, tl in position_traces.items()}

# ---------------------------
# NEW: Pretty printing (no return; pure side-effect printing)
# ---------------------------
def _fmt_row_ints(vals):
    return " ".join(f"{v:2d}" for v in vals)

def _fmt_row_floats(vals, fmt="{:5.1f}"):
    return " ".join(fmt.format(v) for v in vals)

def _fmt_row_labels(labs):
    m = {"steady": "S", "straight": "R", "turn": "T"}  # S=steady, R=straight, T=turn
    return " ".join(m.get(x, "?") for x in labs)

def print_battery_and_motion(battery_traces, position_traces, *, show_energy_constants=True):
    """
    Prints:
      - optional energy constants
      - per-UAV battery timeline (t=0..T)
      - per-UAV motion classification (t=1..T)
    Does NOT alter inputs or returns (you still use compute_battery_for_paths outputs).
    """
    if show_energy_constants:
        try:
            print("\n=== Energy model ===")
            print(f"  Steady: -{E_STEADY:.1f}%  Move: -{E_MOVEMENT:.1f}%  Turn: -{E_TURN:.1f}%")
        except NameError:
            # constants not imported — skip quietly
            pass

    # Battery
    for n in sorted(battery_traces.keys()):
        E = battery_traces[n]
        T = len(E) - 1
        print(f"\nUAV {n+1} battery timeline (%):")
        print("  t : " + _fmt_row_ints(range(0, T+1)))
        print("  E : " + _fmt_row_floats(E))

    # Motion (derived from position_traces so it's in-sync with battery)
    motion_labels = classify_from_timelines(position_traces)
    for n in sorted(motion_labels.keys()):
        labs = motion_labels[n]
        T = len(labs)
        print(f"\nUAV {n+1} motion classification:")
        print("  t : " + _fmt_row_ints(range(1, T+1)))
        print("  L : " + _fmt_row_labels(labs))
        print(f"  counts -> steady:{labs.count('steady')}, straight:{labs.count('straight')}, turn:{labs.count('turn')}")
