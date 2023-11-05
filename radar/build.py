"""Render lua scripts."""

import os
import json


def _render(path, context):
    with open(os.path.join("templates", path)) as f:
        script = f.read()

    with open(os.path.join("scripts", path), 'w') as f:
        f.write(script.format(**context))


def _winpath(p):
    return p.replace("/", "\\").replace("\\", "\\\\")


if __name__ == '__main__':
    with open("config.json") as f:
        cfg = json.load(f)
    cfg["tmpfile"] = os.path.abspath(cfg["tmpfile"])
    cfg["msgfile"] = os.path.abspath(cfg["msgfile"])

    for k, v in cfg.items():
        if isinstance(v, str):
            cfg[k] = _winpath(v)

    os.makedirs("scripts", exist_ok=True)
    for file in os.listdir("templates"):
        _render(file, cfg)
