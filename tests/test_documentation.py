"""The Python examples of the user documentation run and give the results their comments state (#246)

The ``code-block:: python`` directives of a page are executed in order in one namespace, like a reader copying them.
"""

import ast
import importlib.util
import re
import sys
import textwrap
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent

# the pages whose examples run, each after the pages whose names it uses
PAGES = {
    "README.rst": [],
    "docs/0_getting_started.rst": [],
    "docs/1_usage.rst": [],
    # a shortened example using the names defined on the getting started page
    "docs/index.rst": ["docs/0_getting_started.rst"],
}
# pages with Python blocks that do not run, with the reason
NOT_RUN = {
    # examples of errors in pyvisgraph: CI must not depend on external packages (docs/contributing/testing.md)
    "docs/7_benchmark_method.rst": "pyvisgraph",
}
# optional packages and how blocks use them: these blocks are skipped when the package is not installed
OPTIONAL_PACKAGES = {
    "shapely": r"\bshapely\b",
    # the plotting module imports Matplotlib
    "matplotlib": r"\bmatplotlib\b|\bextremitypathfinder\.plotting\b",
}

DIRECTIVE = re.compile(
    r"^(?P<indent>[ \t]*)\.\. (?:code-block|code|sourcecode):: python[ \t]*$"
)
# e.g. "# path: [(4.5, 1.0), (3.0, 7.0), (4.0, 8.5)], length: 7.987..." or "# length: 11.908... (...)"
RESULT_COMMENT = re.compile(
    r"^# (?:path: (?P<path>\[.*\]), )?length: (?P<length>\d+\.\d+)\.\.\."
)


def python_blocks(page: str) -> list[str]:
    """the contents of the Python code directives of a page, dedented (also nested ones, e.g. in a list)"""
    lines = (ROOT / page).read_text(encoding="utf-8").splitlines()
    blocks = []
    for i, line in enumerate(lines):
        match = DIRECTIVE.match(line)
        if match is None:
            continue
        indent = len(match["indent"].expandtabs())
        content = []
        for following in lines[i + 1 :]:
            following_indent = len(following.expandtabs()) - len(
                following.expandtabs().lstrip()
            )
            if following.strip() and following_indent <= indent:
                break
            content.append(following)
        text = textwrap.dedent("\n".join(content))
        # options of the directive (e.g. ":caption:") precede the code
        code = "\n".join(
            line for line in text.splitlines() if not re.match(r"^:\w+:", line)
        )
        blocks.append(code.strip())
    return blocks


def test_every_page_with_examples_is_listed():
    """new pages with Python examples must be run or excluded with a reason"""
    pages = ["README.rst"] + [
        p.relative_to(ROOT).as_posix() for p in sorted((ROOT / "docs").glob("*.rst"))
    ]
    with_examples = {page for page in pages if python_blocks(page)}
    assert with_examples == set(PAGES) | set(NOT_RUN)


def check_result_comment(code: str, namespace: dict) -> bool:
    """compare the result stated in the last line of a block, if any, with the variables ``path`` and ``length``

    Further values in the comment (e.g. "(without clearance: ...)") are not compared.
    """
    lines = code.splitlines()
    stating = [i for i, line in enumerate(lines) if re.match(r"\s*#.*\blength:", line)]
    if not stating:
        return False
    # a result comment the pattern does not recognise would not be compared silently
    assert stating == [len(lines) - 1], (
        "a result comment must be the last line of its block"
    )
    match = RESULT_COMMENT.match(lines[-1].strip())
    assert match is not None, f"unrecognised result comment: {lines[-1]}"
    stated = match["length"]
    decimals = len(stated.split(".")[1])
    # the digits are truncated, not rounded
    assert float(stated) <= namespace["length"] < float(stated) + 10.0**-decimals
    if match["path"] is not None:
        assert namespace["path"] == ast.literal_eval(match["path"])
    return True


def missing_packages(code: str) -> list[str]:
    return [
        name
        for name, usage in OPTIONAL_PACKAGES.items()
        if re.search(usage, code) and importlib.util.find_spec(name) is None
    ]


@pytest.mark.parametrize("page", list(PAGES))
def test_examples_run(page, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)  # examples write files (saved environments, plots)
    monkeypatch.setenv("MPLBACKEND", "Agg")
    try:
        skipped, checked = run_examples(page, {})
    finally:
        if "matplotlib.pyplot" in sys.modules:
            sys.modules["matplotlib.pyplot"].close("all")
    if page in ("README.rst", "docs/1_usage.rst"):
        # the pages stating results
        assert checked > 0
    if skipped:
        pytest.skip(f"blocks need packages not installed: {skipped}")


def run_examples(page: str, namespace: dict) -> tuple[list, int]:
    """run the examples of a page after those of its preludes, the skipped blocks and the number of checked results"""
    for prelude in PAGES[page]:
        for code in python_blocks(prelude):
            if not missing_packages(code):
                exec(compile(code, prelude, "exec"), namespace)  # noqa: S102 (the documentation of this repository)
    skipped = []
    checked = 0
    for code in python_blocks(page):
        missing = missing_packages(code)
        if missing:
            skipped.append(missing)
            continue
        exec(compile(code, page, "exec"), namespace)  # noqa: S102 (the documentation of this repository)
        checked += check_result_comment(code, namespace)
    return skipped, checked
