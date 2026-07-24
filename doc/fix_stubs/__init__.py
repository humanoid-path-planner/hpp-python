"""fix_stubs — Fix Boost.Python .pyi stubs by restoring proper signatures from docstrings.

Public API:
    fix_file(path, setter_overrides=None) -> int
    main()                                      (CLI entry point)
    SETTER_TYPE_OVERRIDES                       (built-in setter type table)
"""

from ._cli import main
from ._fixer import fix_file
from ._overrides import SETTER_TYPE_OVERRIDES

__all__ = ["fix_file", "main", "SETTER_TYPE_OVERRIDES"]
