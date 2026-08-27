#!/usr/bin/env python3
"""Build and run the designer's UI audit.

The designer is a single self-contained HTML file, so the audit stitches its
<script> block together with a fake DOM and drives the real handlers.

    python3 tools/ui/run_ui_audit.py
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PAGE = os.path.join(ROOT, "web", "shape_designer.html")
AUDIT = os.path.join(HERE, "ui_audit.js")
OUT = os.path.join(HERE, "_ui_run.js")

MARK_LOAD = "// ---------------------------------------------------------------- load app"
MARK_TEST = "// ---------------------------------------------------------------- harness"


def main():
    src = open(AUDIT).read()
    pre, rest = src.split(MARK_LOAD)
    post = rest.split(MARK_TEST, 1)[1]
    html = open(PAGE).read()
    js = html.split("<script>")[1].split("</script>")[0].replace('"use strict";', "", 1)
    open(OUT, "w").write(pre + "\n" + js + "\n" + post)
    sys.exit(subprocess.run(["node", OUT]).returncode)


if __name__ == "__main__":
    main()
