"""Prepare and check releases: package version and changelog.

usage:
    python scripts/release.py version            # print the package version
    python scripts/release.py check              # the changelog has a dated section for the package version
    python scripts/release.py prepare patch      # or: minor, major, an explicit version like 2.9.0

``prepare`` sets the new package version with ``poetry version`` and turns the ``Unreleased``
changelog section into the section of the new version, dated today. It requires Poetry.
"""

import argparse
import datetime
import re
import subprocess
import tomllib
from pathlib import Path

PROJECT_DIR = Path(__file__).parent.parent
PYPROJECT_NAME = "pyproject.toml"
CHANGELOG_NAME = "CHANGELOG.rst"

UNRELEASED_TITLE = "Unreleased"
BUMP_RULES = ("major", "minor", "patch")
VERSION_PATTERN = re.compile(r"^(\d+)\.(\d+)\.(\d+)$")


def parse_version(version: str) -> tuple[int, int, int]:
    match = VERSION_PATTERN.match(version)
    if match is None:
        raise ValueError(f"invalid version {version!r}, expected MAJOR.MINOR.PATCH")
    return tuple(int(part) for part in match.groups())


def check_new_version(current: str, new: str) -> None:
    # Note: poetry accepts any version, also lower ones
    if parse_version(new) <= parse_version(current):
        raise ValueError(f"version {new} is not higher than {current}")


def read_version(pyproject: str) -> str:
    try:
        return tomllib.loads(pyproject)["tool"]["poetry"]["version"]
    except KeyError:
        raise ValueError(
            "no version found in the [tool.poetry] table of pyproject.toml"
        )


def poetry_version(project_dir: Path, *args: str) -> str:
    result = subprocess.run(
        ["poetry", "version", "--short", *args],
        cwd=project_dir,
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


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


def prepare(target: str, date: datetime.date, project_dir: Path = PROJECT_DIR) -> str:
    changelog_path = project_dir / CHANGELOG_NAME
    current = read_version((project_dir / PYPROJECT_NAME).read_text(encoding="utf-8"))
    if target not in BUMP_RULES:
        parse_version(target)
    version = poetry_version(project_dir, target, "--dry-run")
    check_new_version(current, version)
    # validate the changelog before changing any file
    new_changelog = release_changelog(
        changelog_path.read_text(encoding="utf-8"), version, date
    )
    poetry_version(project_dir, version)
    changelog_path.write_text(new_changelog, encoding="utf-8")
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
        "target", help=f"one of {', '.join(BUMP_RULES)} or an explicit version"
    )
    prepare_parser.add_argument(
        "--date",
        type=datetime.date.fromisoformat,
        default=datetime.date.today(),
        help="release date (default: today)",
    )
    args = parser.parse_args()

    pyproject = (PROJECT_DIR / PYPROJECT_NAME).read_text(encoding="utf-8")
    try:
        if args.command == "version":
            print(read_version(pyproject))
        elif args.command == "check":
            version = read_version(pyproject)
            changelog = (PROJECT_DIR / CHANGELOG_NAME).read_text(encoding="utf-8")
            date = release_date(changelog, version)
            print(f"CHANGELOG.rst: {version} ({date.isoformat()})")
        else:
            print(prepare(args.target, args.date))
    except ValueError as error:
        parser.exit(1, f"error: {error}\n")
    except subprocess.CalledProcessError as error:
        parser.exit(1, f"error: poetry version failed:\n{error.stderr}")
    except FileNotFoundError:
        parser.exit(1, "error: poetry is required for preparing a release\n")


if __name__ == "__main__":
    main()
