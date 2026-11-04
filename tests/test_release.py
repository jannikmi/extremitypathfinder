import datetime
import json
import os
import shutil
import subprocess

import pytest

from scripts.release import (
    BENCHMARK_RESULTS,
    CHANGELOG_NAME,
    PROJECT_DIR,
    check_new_version,
    outdated_benchmarks,
    prepare,
    read_version,
    release_changelog,
    release_date,
)

PYPROJECT_EXAMPLE = """[project]
name = "extremitypathfinder"
version = "2.8.0"
description = "example"
requires-python = ">=3.12,<4"
dependencies = []

[tool.other]
version = "1.0.0"
"""

CHANGELOG_EXAMPLE = """Changelog
=========

Unreleased
----------

bugfixes:

- fixed something


2.8.0 (2026-09-18)
-------------------

- added something
"""

RELEASE_DATE = datetime.date(2026, 10, 26)
PYPROJECT_NAME = "pyproject.toml"


@pytest.mark.parametrize("new", ["2.8.1", "2.9.0", "3.0.0", "2.10.0"])
def test_check_new_version(new):
    check_new_version("2.8.0", new)


@pytest.mark.parametrize("new", ["2.8.0", "2.7.9", "2.9", "v2.9.0", "2.9.0rc1"])
def test_check_new_version_rejects_invalid_versions(new):
    with pytest.raises(ValueError):
        check_new_version("2.8.0", new)


def test_release_changelog_dates_the_unreleased_section():
    released = release_changelog(CHANGELOG_EXAMPLE, "2.8.1", RELEASE_DATE)

    assert "Unreleased" not in released
    assert "2.8.1 (2026-10-26)\n------------------\n\nbugfixes:" in released
    assert release_date(released, "2.8.1") == RELEASE_DATE
    assert release_date(released, "2.8.0") == datetime.date(2026, 9, 18)


def test_release_changelog_requires_unreleased_entries():
    without_section = CHANGELOG_EXAMPLE.replace("Unreleased\n----------\n", "")
    with pytest.raises(ValueError, match="no 'Unreleased' section"):
        release_changelog(without_section, "2.8.1", RELEASE_DATE)

    empty_section = CHANGELOG_EXAMPLE.replace("bugfixes:\n\n- fixed something\n", "")
    with pytest.raises(ValueError, match="section of CHANGELOG.rst is empty"):
        release_changelog(empty_section, "2.8.1", RELEASE_DATE)


def test_release_date_requires_a_section_for_the_version():
    with pytest.raises(ValueError, match="no section '2.8.1 "):
        release_date(CHANGELOG_EXAMPLE, "2.8.1")


@pytest.fixture
def uv():
    # the tox environment "release" provides uv and sets EXPECT_UV
    if shutil.which("uv") is not None:
        return
    if os.environ.get("EXPECT_UV") == "1":
        pytest.fail("uv is expected (EXPECT_UV=1), but not installed")
    pytest.skip("requires uv")


def git(project_dir, *args) -> str:
    result = subprocess.run(
        ["git", "-c", "user.name=test", "-c", "user.email=test@example.com", *args],
        cwd=project_dir,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def write_benchmark_results(project_dir, **meta):
    """benchmark results of a complete full run of the current commit"""
    meta = {
        "quick": False,
        "completed": "2026-09-17T12:00:00+00:00",
        "git": {"sha": git(project_dir, "rev-parse", "HEAD"), "dirty": False},
        "versions": {"extremitypathfinder": "2.8.0"},
        **meta,
    }
    path = project_dir / BENCHMARK_RESULTS
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"meta": meta}), encoding="utf-8")


def write_lock(project_dir, packages: dict[str, str]):
    entries = "".join(
        f'[[package]]\nname = "{name}"\nversion = "{version}"\n\n'
        for name, version in packages.items()
    )
    (project_dir / "uv.lock").write_text(f"version = 1\n\n{entries}", encoding="utf-8")


@pytest.fixture
def benchmarked_dir(tmp_path):
    """a git repository with up to date benchmark results"""
    if shutil.which("git") is None:
        pytest.skip("requires git")
    git(tmp_path, "init", "--quiet")
    package = tmp_path / "extremitypathfinder"
    package.mkdir()
    (package / "__init__.py").write_text("", encoding="utf-8")
    (tmp_path / "benchmarks").mkdir()
    (tmp_path / "benchmarks" / "run.py").write_text("", encoding="utf-8")
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "--quiet", "-m", "package")
    write_benchmark_results(tmp_path)
    git(tmp_path, "add", ".")
    git(tmp_path, "commit", "--quiet", "-m", "benchmark results")
    return tmp_path


@pytest.fixture
def project_dir(uv, benchmarked_dir):
    (benchmarked_dir / PYPROJECT_NAME).write_text(PYPROJECT_EXAMPLE, encoding="utf-8")
    (benchmarked_dir / CHANGELOG_NAME).write_text(CHANGELOG_EXAMPLE, encoding="utf-8")
    return benchmarked_dir


def test_read_version_reads_the_package_version(project_dir):
    assert read_version(project_dir) == "2.8.0"


def test_repository_changelog_matches_package_version(uv):
    # the same check guards the release tagging in CI
    release_date(
        (PROJECT_DIR / CHANGELOG_NAME).read_text(encoding="utf-8"), read_version()
    )


@pytest.mark.parametrize(
    ("target", "expected"),
    [("patch", "2.8.1"), ("minor", "2.9.0"), ("major", "3.0.0"), ("2.10.0", "2.10.0")],
)
def test_prepare_sets_version_and_dates_changelog(project_dir, target, expected):
    assert prepare(target, RELEASE_DATE, project_dir) == expected

    pyproject = (project_dir / PYPROJECT_NAME).read_text(encoding="utf-8")
    changelog = (project_dir / CHANGELOG_NAME).read_text(encoding="utf-8")
    assert read_version(project_dir) == expected
    assert 'version = "1.0.0"' in pyproject
    assert release_date(changelog, expected) == RELEASE_DATE
    # the lock file records the version of the project as well
    lock = (project_dir / "uv.lock").read_text(encoding="utf-8")
    assert f'name = "extremitypathfinder"\nversion = "{expected}"' in lock


@pytest.mark.parametrize("target", ["2.7.0", "2.9", "prerelease"])
def test_prepare_rejects_invalid_targets_without_changes(project_dir, target):
    with pytest.raises(ValueError):
        prepare(target, RELEASE_DATE, project_dir)

    assert (project_dir / PYPROJECT_NAME).read_text(
        encoding="utf-8"
    ) == PYPROJECT_EXAMPLE
    assert (project_dir / CHANGELOG_NAME).read_text(
        encoding="utf-8"
    ) == CHANGELOG_EXAMPLE
    assert not (project_dir / "uv.lock").exists()


def test_prepare_requires_unreleased_entries_without_changes(project_dir):
    changelog = CHANGELOG_EXAMPLE.replace("Unreleased\n----------\n", "")
    (project_dir / CHANGELOG_NAME).write_text(changelog, encoding="utf-8")

    with pytest.raises(ValueError, match="no 'Unreleased' section"):
        prepare("patch", RELEASE_DATE, project_dir)

    assert (project_dir / PYPROJECT_NAME).read_text(
        encoding="utf-8"
    ) == PYPROJECT_EXAMPLE


def test_prepare_requires_current_benchmarks_without_changes(project_dir):
    (project_dir / "extremitypathfinder" / "__init__.py").write_text(
        "x = 1\n", encoding="utf-8"
    )

    with pytest.raises(ValueError, match="benchmark results are outdated"):
        prepare("patch", RELEASE_DATE, project_dir)

    assert (project_dir / PYPROJECT_NAME).read_text(
        encoding="utf-8"
    ) == PYPROJECT_EXAMPLE
    assert (project_dir / CHANGELOG_NAME).read_text(
        encoding="utf-8"
    ) == CHANGELOG_EXAMPLE


def test_benchmarks_up_to_date(benchmarked_dir):
    write_benchmark_results(
        benchmarked_dir, versions={"extremitypathfinder": "2.8.0", "numpy": "2.5.4"}
    )
    write_lock(benchmarked_dir, {"extremitypathfinder": "2.9.0", "numpy": "2.5.4"})
    # other files, the version of the package and a later commit do not matter
    (benchmarked_dir / "README.rst").write_text("changed", encoding="utf-8")
    git(benchmarked_dir, "add", ".")
    git(benchmarked_dir, "commit", "--quiet", "-m", "docs")
    assert outdated_benchmarks(benchmarked_dir) == []


@pytest.mark.parametrize("committed", [False, True])
@pytest.mark.parametrize(
    "path", ["extremitypathfinder/__init__.py", "benchmarks/run.py"]
)
def test_benchmarks_outdated_by_changed_code(benchmarked_dir, path, committed):
    (benchmarked_dir / path).write_text("x = 1\n", encoding="utf-8")
    if committed:
        git(benchmarked_dir, "commit", "--quiet", "-am", "change")
    reasons = outdated_benchmarks(benchmarked_dir)
    assert len(reasons) == 1
    assert path in reasons[0]


@pytest.mark.parametrize(
    ("locked", "measured", "expected"),
    [
        ({"numpy": "2.5.5"}, "2.5.4", "numpy: measured 2.5.4, locked 2.5.5"),
        # e.g. a dependency dropped from the extra (#185)
        ({}, "2.5.4", "numpy: measured 2.5.4, locked not in uv.lock"),
        ({"numpy": "2.5.4"}, None, "numpy: measured not installed, locked 2.5.4"),
    ],
)
def test_benchmarks_outdated_by_dependency_versions(
    benchmarked_dir, locked, measured, expected
):
    write_benchmark_results(
        benchmarked_dir, versions={"extremitypathfinder": "2.8.0", "numpy": measured}
    )
    write_lock(benchmarked_dir, locked)
    assert outdated_benchmarks(benchmarked_dir) == [expected]


@pytest.mark.parametrize(
    ("meta", "expected"),
    [
        ({"quick": True}, "not from a complete full run"),
        ({"completed": None}, "not from a complete full run"),
        ({"git": {"sha": None, "dirty": False}}, "do not name the measured commit"),
        ({"git": {"sha": "0" * 40, "dirty": False}}, "cannot compare"),
    ],
)
def test_benchmarks_outdated_by_run(benchmarked_dir, meta, expected):
    write_benchmark_results(benchmarked_dir, **meta)
    reasons = outdated_benchmarks(benchmarked_dir)
    assert len(reasons) == 1
    assert expected in reasons[0]


def test_benchmarks_outdated_by_uncommitted_measurement(benchmarked_dir):
    sha = git(benchmarked_dir, "rev-parse", "HEAD")
    write_benchmark_results(benchmarked_dir, git={"sha": sha, "dirty": True})
    assert outdated_benchmarks(benchmarked_dir) == [
        f"the results were measured with uncommitted changes (commit {sha})"
    ]


def test_benchmarks_missing(tmp_path):
    assert outdated_benchmarks(tmp_path) == [f"{BENCHMARK_RESULTS} does not exist"]
