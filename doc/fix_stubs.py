#!/usr/bin/env python3
"""Backward-compatible entry point — logic lives in the fix_stubs/ package.

Usage (unchanged):
    python fix_stubs.py path/to/site-packages/pyhpp/
    python fix_stubs.py some/module/bindings.pyi another.pyi
    python fix_stubs.py --setter-overrides overrides.json path/to/pyhpp/

Or via the package:
    python -m fix_stubs ...
"""

from fix_stubs._cli import main

if __name__ == "__main__":
    main()
