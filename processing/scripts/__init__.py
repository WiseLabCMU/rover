"""Data Processing (Python) Scripts.

::
     ______  _____  _    _ _______  ______
    |_____/ |     |  \  /  |______ |_____/
    |    \_ |_____|   \/   |______ |    \_
    Radar Range-Doppler-Azimuth Processing

.
"""

from . import fft, trajectory, report, map, cfar

commands = {
    "fft": fft,
    "trajectory": trajectory,
    "report": report,
    "map": map,
    "cfar": cfar
}
