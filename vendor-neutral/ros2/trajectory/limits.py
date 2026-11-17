def _validate_limits(limits_deg, joints):
    for j in joints:
        if j not in limits_deg:
            raise ValueError(f"robot.yaml: joint '{j}' missing limits_deg")
        lim = limits_deg[j]
        if not (isinstance(lim, (list, tuple)) and len(lim) == 2):
            raise ValueError(f"joint '{j}': limits_deg must be [lower, upper]")
        lo, hi = lim
        if lo >= hi:
            raise ValueError(f"joint '{j}': lower {lo}° >= upper {hi}°")
        if hi - lo > 720.0:
            raise ValueError(f"joint '{j}': range {hi - lo:.1f}° is implausibly large")
