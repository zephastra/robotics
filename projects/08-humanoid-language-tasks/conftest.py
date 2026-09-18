"""Make the src/ layout importable when tests run without installation.

This keeps `pytest` runnable from a fresh checkout even before
`bash scripts/setup.sh` installs the project in editable mode.
"""

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent / "src"
if SRC.is_dir() and str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
