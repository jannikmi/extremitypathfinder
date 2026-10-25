import datetime
import shutil

import pytest

from scripts.release import (
    CHANGELOG_NAME,
    PROJECT_DIR,
    check_new_version,
    prepare,
    read_version,
    release_changelog,
    release_date,
)

PYPROJECT_EXAMPLE = """[tool.poetry]
name = "extremitypathfinder"
version = "2.8.0"
description = "example"
authors = ["example <example@example.com>"]

[tool.poetry.dependencies]
python = ">=3.12,<4"

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

requires_poetry = pytest.mark.skipif(
    shutil.which("poetry") is None, reason="requires poetry"
)


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
def project_dir(tmp_path):
    if shutil.which("poetry") is None:
        pytest.skip("requires poetry")
    (tmp_path / PYPROJECT_NAME).write_text(PYPROJECT_EXAMPLE, encoding="utf-8")
    (tmp_path / CHANGELOG_NAME).write_text(CHANGELOG_EXAMPLE, encoding="utf-8")
    return tmp_path


def test_read_version_reads_the_package_version(project_dir):
    assert read_version(project_dir) == "2.8.0"


@requires_poetry
def test_repository_changelog_matches_package_version():
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


def test_prepare_requires_unreleased_entries_without_changes(project_dir):
    changelog = CHANGELOG_EXAMPLE.replace("Unreleased\n----------\n", "")
    (project_dir / CHANGELOG_NAME).write_text(changelog, encoding="utf-8")

    with pytest.raises(ValueError, match="no 'Unreleased' section"):
        prepare("patch", RELEASE_DATE, project_dir)

    assert (project_dir / PYPROJECT_NAME).read_text(
        encoding="utf-8"
    ) == PYPROJECT_EXAMPLE
