import warnings

warnings.filterwarnings(
    "ignore",
    message="to-Python converter .* already registered",
    category=RuntimeWarning,
)

import eigenpy  # noqa: F401
