#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
/usr/bin/python3 - "$ROOT" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
applications = Path.home()/'.local/share/applications'
applications.mkdir(parents=True, exist_ok=True)
launcher = applications/'imt342-abb-real.desktop'
launcher.write_text('[Desktop Entry]\nType=Application\nName=ABB real en RViz\n'
    'Comment=Seguir los seis ejes reales del ABB\n'
    f'Exec=/usr/bin/bash "{root}/abrir_real.sh"\n'
    'Icon=applications-engineering\nTerminal=true\nCategories=Education;Science;\n')
launcher.chmod(0o755)
limits_launcher = applications/'imt342-abb-limites.desktop'
limits_launcher.write_text('[Desktop Entry]\nType=Application\nName=ABB limites y porcentajes\n'
    f'Exec=/usr/bin/bash "{root}/configurar_limites.sh"\n'
    'Icon=applications-engineering\nTerminal=true\nCategories=Education;Science;\n')
limits_launcher.chmod(0o755)
panel_launcher = applications/'imt342-abb-panel.desktop'
panel_launcher.write_text('[Desktop Entry]\nType=Application\nName=ABB panel didactico\n'
    f'Exec=/usr/bin/bash "{root}/abrir_panel.sh"\n'
    'Icon=applications-engineering\nTerminal=true\nCategories=Education;Science;\n')
panel_launcher.chmod(0o755)
home_command = Path.home()/'abrirabb_real'
home_command.write_text('#!/usr/bin/env bash\nexec /usr/bin/bash '+repr(str(root/'abrir_real.sh'))+' "$@"\n')
home_command.chmod(0o755)
print('Listo: ABB real en RViz. Desde tu carpeta personal: bash ~/abrirabb_real')
PY
