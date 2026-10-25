"""Prepare and check releases: package version and changelog.

usage:
    python scripts/release.py version            # print the package version
    python scripts/release.py check              # the changelog has a dated section for the package version
    python scripts/release.py prepare patch      # or: minor, major, an explicit version like 2.9.0

``prepare`` sets the new package version in ``pyproject.toml`` and turns the ``Unreleased``
changelog section into the section of the new version, dated today.
"""

import argparse
import datetime
import re
from pathlib import Path

PROJECT_DIR = Path(__file__).parent.parent
PYPROJECT = PROJECT_DIR / "pyproject.toml"
CHANGELOG = PROJECT_DIR / "CHANGELOG.rst"

UNRELEASED_TITLE = "Unreleased"
BUMP_LEVELS = ("major", "minor", "patch")
VERSION_PATTERN = re.compile(r"^(\d+)\.(\d+)\.(\d+)$")
# the package version is the first assignment in the [tool.poetry] table
PYPROJECT_VERSION_PATTERN = re.compile(
    r'^(\[tool\.poetry\]\n(?:(?!\[).*\n)*?)version = "([^"]+)"$', re.MULTILINE
)


def parse_version(version: str) -> tuple[int, int, int]:
    match = VERSION_PATTERN.match(version)
    if match is None:
        raise ValueError(f"invalid version {version!r}, expected MAJOR.MINOR.PATCH")
    return tuple(int(part) for part in match.groups())


def bump_version(current: str, target: str) -> str:
    """Return the next version, given a bump level or an explicit version."""
    major, minor, patch = parse_version(current)
    if target == "major":
        return f"{major + 1}.0.0"
    if target == "minor":
        return f"{major}.{minor + 1}.0"
    if target == "patch":
        return f"{major}.{minor}.{patch + 1}"
    if parse_version(target) <= (major, minor, patch):
        raise ValueError(f"version {target} is not higher than {current}")
    return target


def read_version(pyproject: str) -> str:
    match = PYPROJECT_VERSION_PATTERN.search(pyproject)
    if match is None:
        raise ValueError(
            "no version found in the [tool.poetry] table of pyproject.toml"
        )
    return match.group(2)


def set_version(pyproject: str, version: str) -> str:
    read_version(pyproject)  # fail early without a version
    return PYPROJECT_VERSION_PATTERN.sub(
        lambda match: f'{match.group(1)}version = "{version}"', pyproject, count=1
    )


def _section_title_pattern(title: str) -> re.Pattern:
    # RST section: the title line followed by an underline of "-"
    return re.compile(rf"^{re.escape(title)}\n-+\n", re.MULTILINE)


def release_date(changelog: str, version: str) -> datetime.date:
    """Return the date of the changelog section of the given version."""
    match = re.search(
        rf"^{re.escape(version)} \((\d{{4}}-\d{{2}}-\d{{2}})\)\n-+\n",
        changelog,
        re.MULTILINE,
    )
    if match is None:
        raise ValueError(
            f"CHANGELOG.rst has no section '{version} (YYYY-MM-DD)' for the package version"
        )
    return datetime.date.fromisoformat(match.group(1))


def release_changelog(changelog: str, version: str, date: datetime.date) -> str:
    """Turn the 'Unreleased' section into the section of the given version."""
    unreleased = _section_title_pattern(UNRELEASED_TITLE).search(changelog)
    if unreleased is None:
        raise ValueError(f"CHANGELOG.rst has no '{UNRELEASED_TITLE}' section")
    next_section = re.compile(r"^.+\n-+\n", re.MULTILINE).search(
        changelog, unreleased.end()
    )
    section_end = next_section.start() if next_section else len(changelog)
    if not changelog[unreleased.end() : section_end].strip():
        raise ValueError(f"the '{UNRELEASED_TITLE}' section of CHANGELOG.rst is empty")
    title = f"{version} ({date.isoformat()})"
    return (
        changelog[: unreleased.start()]
        + f"{title}\n{'-' * len(title)}\n"
        + changelog[unreleased.end() :]
    )


def prepare(target: str, date: datetime.date) -> str:
    pyproject = PYPROJECT.read_text(encoding="utf-8")
    changelog = CHANGELOG.read_text(encoding="utf-8")
    version = bump_version(read_version(pyproject), target)
    # compute both files before writing any of them
    new_changelog = release_changelog(changelog, version, date)
    new_pyproject = set_version(pyproject, version)
    CHANGELOG.write_text(new_changelog, encoding="utf-8")
    PYPROJECT.write_text(new_pyproject, encoding="utf-8")
    return version


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("version", help="print the package version")
    commands.add_parser(
        "check",
        help="check that the changelog has a dated section for the package version",
    )
    prepare_parser = commands.add_parser(
        "prepare",
        help="set the new version and date the 'Unreleased' changelog section",
    )
    prepare_parser.add_argument(
        "target", help=f"one of {', '.join(BUMP_LEVELS)} or an explicit version"
    )
    prepare_parser.add_argument(
        "--date",
        type=datetime.date.fromisoformat,
        default=datetime.date.today(),
        help="release date (default: today)",
    )
    args = parser.parse_args()

    try:
        if args.command == "version":
            print(read_version(PYPROJECT.read_text(encoding="utf-8")))
        elif args.command == "check":
            version = read_version(PYPROJECT.read_text(encoding="utf-8"))
            date = release_date(CHANGELOG.read_text(encoding="utf-8"), version)
            print(f"CHANGELOG.rst: {version} ({date.isoformat()})")
        else:
            print(prepare(args.target, args.date))
    except ValueError as error:
        parser.exit(1, f"error: {error}\n")


if __name__ == "__main__":
    main()
