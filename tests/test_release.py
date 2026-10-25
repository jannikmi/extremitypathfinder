import datetime

import pytest

from scripts.release import (
    CHANGELOG,
    PYPROJECT,
    bump_version,
    read_version,
    release_changelog,
    release_date,
    set_version,
)

PYPROJECT_EXAMPLE = """[tool.poetry]
name = "extremitypathfinder"
version = "2.8.0"

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


@pytest.mark.parametrize(
    ("target", "expected"),
    [
        ("patch", "2.8.1"),
        ("minor", "2.9.0"),
        ("major", "3.0.0"),
        ("2.10.0", "2.10.0"),
    ],
)
def test_bump_version(target, expected):
    assert bump_version("2.8.0", target) == expected


@pytest.mark.parametrize("target", ["2.8.0", "2.7.9", "2.9", "v2.9.0", "2.9.0rc1"])
def test_bump_version_rejects_invalid_targets(target):
    with pytest.raises(ValueError):
        bump_version("2.8.0", target)


def test_read_and_set_version_only_change_the_package_version():
    assert read_version(PYPROJECT_EXAMPLE) == "2.8.0"

    updated = set_version(PYPROJECT_EXAMPLE, "2.8.1")

    assert read_version(updated) == "2.8.1"
    assert updated == PYPROJECT_EXAMPLE.replace(
        'version = "2.8.0"', 'version = "2.8.1"'
    )


def test_release_changelog_dates_the_unreleased_section():
    released = release_changelog(
        CHANGELOG_EXAMPLE, "2.8.1", datetime.date(2026, 10, 26)
    )

    assert "Unreleased" not in released
    assert "2.8.1 (2026-10-26)\n------------------\n\nbugfixes:" in released
    assert release_date(released, "2.8.1") == datetime.date(2026, 10, 26)
    assert release_date(released, "2.8.0") == datetime.date(2026, 9, 18)


def test_release_changelog_requires_unreleased_entries():
    without_section = CHANGELOG_EXAMPLE.replace("Unreleased\n----------\n", "")
    with pytest.raises(ValueError, match="no 'Unreleased' section"):
        release_changelog(without_section, "2.8.1", datetime.date(2026, 10, 26))

    empty_section = CHANGELOG_EXAMPLE.replace("bugfixes:\n\n- fixed something\n", "")
    with pytest.raises(ValueError, match="section of CHANGELOG.rst is empty"):
        release_changelog(empty_section, "2.8.1", datetime.date(2026, 10, 26))


def test_release_date_requires_a_section_for_the_version():
    with pytest.raises(ValueError, match="no section '2.8.1 "):
        release_date(CHANGELOG_EXAMPLE, "2.8.1")


def test_repository_changelog_matches_package_version():
    # the same check guards the release tagging in CI
    version = read_version(PYPROJECT.read_text(encoding="utf-8"))
    release_date(CHANGELOG.read_text(encoding="utf-8"), version)
