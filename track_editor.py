"""Shortcut: runs maps/track_editor.py so `python track_editor.py` works from the project folder."""
import os
import runpy
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
for _sub in ("", "aruco", "fusion", "maps", "ui"):
    sys.path.insert(0, os.path.join(_ROOT, _sub))
runpy.run_path(os.path.join(_ROOT, "maps", "track_editor.py"), run_name="__main__")
