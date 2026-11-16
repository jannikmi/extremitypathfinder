import json
import subprocess
from pathlib import Path

import pytest

PROJECT_DIR = Path(__file__).parent.parent
EXAMPLE_FILE = PROJECT_DIR / "example.json"


@pytest.mark.parametrize(
    "cmd",
    [
        f"extremitypathfinder {EXAMPLE_FILE} -s 2.5 3.2 -g 7.9 6.8",
    ],
)
def test_main(cmd: list[str]):
    res = subprocess.getoutput(cmd)
    assert not res.endswith("command not found"), "package not installed"
    splits = res.split(" ")
    length = float(splits[-1])
    print("length:", length)
    path = " ".join(splits[:-1])
    print("path:", path)


def test_main_validates_the_input(tmp_path):
    """regression (#243): invalid input is rejected instead of giving wrong paths"""
    invalid = {
        "boundary": [[0, 0], [10, 0], [10, 10], [0, 10]],
        "holes": [[[4, 4], [6, 6], [6, 4], [4, 6]]],  # self-intersecting
    }
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(invalid))
    res = subprocess.run(
        ["extremitypathfinder", str(path), "-s", "1", "5", "-g", "9", "5"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert res.returncode != 0
    assert "self intersections" in res.stderr
