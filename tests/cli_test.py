import subprocess
from pathlib import Path
from typing import List

import pytest

PROJECT_DIR = Path(__file__).parent.parent
EXAMPLE_FILE = PROJECT_DIR / "example.json"


@pytest.mark.parametrize(
    "cmd",
    [
        f"extremitypathfinder {EXAMPLE_FILE} -s 2.5 3.2 -g 7.9 6.8",
    ],
)
def test_main(cmd: List[str]):
    res = subprocess.getoutput(cmd)
    assert not res.endswith("command not found"), "package not installed"
    splits = res.split(" ")
    length = float(splits[-1])
    print("length:", length)
    path = " ".join(splits[:-1])
    print("path:", path)
