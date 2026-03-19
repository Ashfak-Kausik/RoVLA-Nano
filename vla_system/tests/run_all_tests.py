#!/usr/bin/env python3
"""
VLA Test Runner
---------------
Runs all test suites in sequence.

Usage:
  python3 tests/run_all_tests.py
  ./tests/run_all_tests.py
"""

import sys
import os
import subprocess

TESTS = [
    ("Language Parser",  "tests/test_language_parser.py"),
    ("Integration",      "tests/test_integration.py"),
]

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def main():
    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║          VLA System — Full Test Suite                ║")
    print("╚══════════════════════════════════════════════════════╝")

    overall_ok = True
    for name, path in TESTS:
        print(f"\n{'─'*56}")
        print(f"  Running: {name}")
        print(f"{'─'*56}")
        result = subprocess.run(
            [sys.executable, os.path.join(ROOT, path)],
            cwd=ROOT,
        )
        if result.returncode != 0:
            overall_ok = False

    print()
    print("╔══════════════════════════════════════════════════════╗")
    if overall_ok:
        print("║  ✓  All test suites passed.                          ║")
    else:
        print("║  ✗  One or more test suites FAILED.                  ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()
    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
