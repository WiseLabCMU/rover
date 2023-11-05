"""Launch mmWave Studio."""

import os
import json
import subprocess


with open("config.json") as f:
    cfg = json.load(f)

subprocess.Popen([
    os.path.join(cfg["mmwave_studio", "RunTime", "mmWaveStudio.exe"]),
    "/lua", "scripts\\server.lua"])
