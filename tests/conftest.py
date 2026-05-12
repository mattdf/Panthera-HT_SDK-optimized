import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

if importlib.util.find_spec("Panthera_lib") is None:
    sys.path.append(str(ROOT / "panthera_python" / "scripts"))
