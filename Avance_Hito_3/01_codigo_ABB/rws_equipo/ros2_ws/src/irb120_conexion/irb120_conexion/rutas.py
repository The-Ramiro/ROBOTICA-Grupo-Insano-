"""Recursos compartidos: modelo Hito 1 y raíz del repositorio fuente."""
from pathlib import Path
import sys


def project_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "model_manifest.yaml").is_file() and (parent / "ros2_ws").is_dir():
            return parent
    return None


PROJECT = project_root()
if PROJECT is not None:
    model_source = PROJECT / "ros2_ws/src/irb120_hito1"
    if str(model_source) not in sys.path:
        sys.path.insert(0, str(model_source))
    PACKAGE = model_source
else:
    from ament_index_python.packages import get_package_share_directory
    PACKAGE = Path(get_package_share_directory("irb120_hito1"))

SOURCE_PACKAGE = Path(__file__).resolve().parent
DEFAULT_OUTPUT = (PROJECT if PROJECT is not None else Path.cwd()) / "salidas"
