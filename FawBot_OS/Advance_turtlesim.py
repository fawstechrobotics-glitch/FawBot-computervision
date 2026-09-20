"""Backward compatibility wrapper for FawBot OS.

This file ensures that running `python Advance_turtlesim.py` seamlessly
launches the refactored modular FawBot application.
"""
import sys
import os

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from main import main

if __name__ == "__main__":
    main()