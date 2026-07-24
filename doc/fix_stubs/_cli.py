"""Command-line interface for fix_stubs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ._fixer import fix_file
from ._overrides import load_method_overrides, load_setter_overrides


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fix Boost.Python .pyi stubs: restore proper signatures from docstrings"
    )
    parser.add_argument(
        "paths",
        nargs="+",
        type=Path,
        help=".pyi files or directories to fix (directories are searched recursively)",
    )
    parser.add_argument(
        "--setter-overrides",
        type=Path,
        default=None,
        help=(
            "Optional JSON file with additional/overriding setter type mappings "
            '(flat object: {"ClassName.propName": "type", ...}). '
            "Merged on top of the built-in SETTER_TYPE_OVERRIDES table (external entries win)."
        ),
    )
    parser.add_argument(
        "--method-overrides",
        type=Path,
        default=None,
        help=(
            "Optional JSON file with additional/overriding method signature mappings "
            '({"ClassName.method": {"ret": "Type", "params": [["name", "Type"], ...]}}). '
            "Merged on top of the built-in METHOD_OVERRIDES table (external entries win)."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "Write fixed stubs to this directory instead of modifying in-place. "
            "The input directory structure is preserved relative to each input path."
        ),
    )
    args = parser.parse_args()

    setter_overrides = load_setter_overrides(args.setter_overrides)
    method_overrides = load_method_overrides(args.method_overrides)

    total = 0
    total_files = 0
    for p in args.paths:
        if not p.exists():
            print(f"warning: path not found, skipping: {p}", file=sys.stderr)
            continue
        root = p if p.is_dir() else p.parent
        files = sorted(p.rglob("*.pyi")) if p.is_dir() else [p]
        for f in files:
            output_path = (
                args.output_dir / f.relative_to(root)
                if args.output_dir is not None
                else None
            )
            try:
                n = fix_file(
                    f,
                    output_path=output_path,
                    setter_overrides=setter_overrides,
                    method_overrides=method_overrides,
                )
            except Exception as e:
                print(f"error: failed to fix {f}: {e}", file=sys.stderr)
                continue
            total_files += 1
            if n:
                print(f"{f}: {n} signature(s) fixed", file=sys.stderr)
            total += n
    print(
        f"Total: {total} signature(s) fixed across {total_files} file(s)",
        file=sys.stderr,
    )
