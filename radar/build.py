"""Render lua scripts."""

import os
from argparse import ArgumentParser


def _render(path, context):
    with open(os.path.join("templates", path)) as f:
        script = f.read()

    with open(os.path.join("scripts", path), 'w') as f:
        f.write(script.format(**context))


def _winpath(p):
    return p.replace("/", "\\").replace("\\", "\\\\")


if __name__ == '__main__':
    p = ArgumentParser(description="Render lua scripts from templates.")
    p.add_argument(
        "--com", type=str, default="4",
        help="XDS110 Class Application/User UART COM Port.")
    p.add_argument(
        "--tmpfile", type=str, default="./tmp.bin",
        help="Data recording temporary file to pass to mmWave studio.")
    p.add_argument(
        "--mmwave_studio", type=str,
        default="C:\\ti\\mmwave_studio_02_01_01_00",
        help="mmWave Studio install directory.")
    args = p.parse_args()

    context = {
        "com": args.com,
        "tmpfile": _winpath(os.path.abspath(args.tmpfile)),
        "mmwave_studio": _winpath(args.mmwave_studio)
    }

    os.makedirs("scripts", exist_ok=True)
    for file in os.listdir("templates"):
        _render(file, context)
