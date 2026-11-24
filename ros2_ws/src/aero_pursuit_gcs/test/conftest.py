"""Tests import the package from this source tree, not a stale copy in install/ (a sourced
workspace puts install/ first on sys.path, and the GCS is not built with --symlink-install)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
