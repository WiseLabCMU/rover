"""Launch mmWave Studio."""

import os
import json
import subprocess


with open("config.json") as f:
    cfg = json.load(f)

script = os.path.join(os.path.dirname(__file__), "scripts", "server.lua")
cwd = os.getcwd()
os.chdir(os.path.join(cfg["mmwave_studio"], "mmWaveStudio", "RunTime"))
subprocess.Popen(["mmWaveStudio.exe", "/lua", script])
os.chdir(cwd)
