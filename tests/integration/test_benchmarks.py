#!/usr/bin/env python3
#
# Copyright (c) 2025 CNRS
# Author: Paul Sardin
#
# Benchmark test runner - runs all benchmarks with N=1 to verify they work.

import subprocess
import sys
from pathlib import Path


def main():
    folder = Path(__file__).parent

    # List of benchmark files to run (excluding this file and utilities)
    excluded = {Path(__file__).name, "benchmark_utils.py", "__init__.py"}
    benchmarks = sorted(
        script for script in folder.glob("*.py")
        if script.name not in excluded
    )

    print(f"Running {len(benchmarks)} benchmarks...")

    failed = []
    for script in benchmarks:
        print(f"\n{'='*60}")
        print(f"Running: {script.name}")
        print("=" * 60)

        result = subprocess.run(
            [sys.executable, str(script), "-N", "1"],
            cwd=str(folder),
        )

        if result.returncode != 0:
            failed.append(script.name)
            print(f"FAILED: {script.name} (exit code {result.returncode})")

    print(f"\n{'='*60}")
    print("SUMMARY")
    print("=" * 60)
    print(f"Total benchmarks: {len(benchmarks)}")
    print(f"Passed: {len(benchmarks) - len(failed)}")
    print(f"Failed: {len(failed)}")

    if failed:
        print(f"\nFailed benchmarks: {', '.join(failed)}")
        sys.exit(1)
    else:
        print("\nAll benchmarks passed!")


if __name__ == "__main__":
    main()
