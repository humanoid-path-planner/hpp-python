#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path


def main():
    folder = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")

    for script in sorted(folder.glob("*.py")):
        if script.name == Path(__file__).name:
            continue
        print(f"\n{'=' * 60}\nRunning: {script.name}\n{'=' * 60}")
        subprocess.run([sys.executable, str(script), "-N", "1"])


if __name__ == "__main__":
    main()
