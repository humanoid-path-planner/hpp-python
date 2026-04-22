import numpy as np


def describe_error(config_projector, q):
    """Compute constraint errors and map each component to its constraint.

    Args:
        config_projector: A ConfigProjector instance.
        q: Configuration vector to check.

    Returns:
        Tuple of (entries, satisfied) where entries is a list of dicts with:
            name: constraint function name
            error: numpy array of error values
            norm: L2 norm of the error
            kind: "implicit" or "explicit"
            priority: priority level (implicit) or None (explicit)
            satisfied: whether norm < threshold
    """
    solver = config_projector.solver()
    threshold = config_projector.errorThreshold()

    raw = solver.describeError(q)

    entries = []
    for name, error, kind, priority in raw:
        error = np.array(error)
        norm = float(np.linalg.norm(error))
        entries.append(
            {
                "name": name,
                "error": error,
                "norm": norm,
                "kind": kind,
                "priority": priority if priority >= 0 else None,
                "satisfied": norm < threshold,
            }
        )

    satisfied = all(e["satisfied"] for e in entries)
    return entries, satisfied


def print_error(config_projector, q):
    """Print a human-readable breakdown of constraint errors."""
    entries, satisfied = describe_error(config_projector, q)
    threshold = config_projector.errorThreshold()

    print(f"Overall satisfied: {satisfied}  (threshold: {threshold:.0e})")
    print(
        f"{'Constraint':<50} {'Kind':<10} {'Pri':<5} {'Norm':>12} {'OK?':>5}"
    )
    print("-" * 85)
    for e in entries:
        pri = str(e["priority"]) if e["priority"] is not None else "-"
        ok = "yes" if e["satisfied"] else "NO"
        print(
            f"{e['name']:<50} {e['kind']:<10} {pri:<5} "
            f"{e['norm']:>12.6e} {ok:>5}"
        )
