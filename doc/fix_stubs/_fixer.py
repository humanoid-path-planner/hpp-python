"""Orchestrate parsing, import merging, and rendering for a single .pyi file."""

from __future__ import annotations

from pathlib import Path

from ._imports import collect_required_imports, merge_imports
from ._parser import extract_tree
from ._renderer import render_file


def fix_file(
    path: Path,
    setter_overrides: dict[str, str] | None = None,
    method_overrides: dict | None = None,
) -> int:
    """Fix a single .pyi file in-place. Returns the number of signatures fixed."""
    header, tree = extract_tree(
        path,
        setter_overrides=setter_overrides,
        method_overrides=method_overrides,
    )

    required_modules = collect_required_imports(tree)
    header = merge_imports(header, required_modules)

    fixed = 0
    for cls in tree:
        for fn in cls.functions:
            if fn.from_docstring:
                fixed += 1
            elif fn.setter_overloads:
                fixed += 1

    new_content = render_file(header, tree)
    path.write_text(new_content)

    return fixed
