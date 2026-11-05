"""Prepare and check releases: package version, changelog and citation metadata.

usage:
    python scripts/release.py version            # print the package version
    python scripts/release.py check              # the changelog has a dated section for the package version,
                                                 # CITATION.cff names the version and its date
    python scripts/release.py benchmarks         # the published benchmark results are up to date
    python scripts/release.py benchmarks --if-version-changed-since <commit>   # only for a new version
    python scripts/release.py prepare patch      # or: minor, major, an explicit version like 2.9.0

``prepare`` sets the new package version and turns the ``Unreleased`` changelog section
into the section of the new version, dated today, and sets the version and the date in ``CITATION.cff``.
It refuses to prepare a release while the published benchmark results are outdated.
The version is read and set with ``uv version``, so all commands require uv (and git for the benchmark check).
"""

import argparse
import datetime
import json
import re
import subprocess
import tomllib
from pathlib import Path

PROJECT_DIR = Path(__file__).parent.parent
CHANGELOG_NAME = "CHANGELOG.rst"
CITATION_NAME = "CITATION.cff"
BENCHMARK_RESULTS = "benchmarks/results/benchmark.json"
# the code which determines the benchmark results (cf. MEASURED_CODE in benchmarks/run.py).
# The dependencies are compared by their versions instead: pyproject.toml and uv.lock change with every release.
BENCHMARKED_CODE = ("extremitypathfinder", "benchmarks/*.py")
RERUN_HINT = (
    "rerun the full benchmark and commit its results "
    "(docs/contributing/performance.md, 'Running the benchmarks')"
)

UNRELEASED_TITLE = "Unreleased"
BUMP_RULES = ("major", "minor", "patch")
VERSION_PATTERN = re.compile(r"^(\d+)\.(\d+)\.(\d+)$")


def parse_version(version: str) -> tuple[int, int, int]:
    match = VERSION_PATTERN.match(version)
    if match is None:
        raise ValueError(f"invalid version {version!r}, expected MAJOR.MINOR.PATCH")
    return tuple(int(part) for part in match.groups())


def check_new_version(current: str, new: str) -> None:
    # Note: uv accepts any version, also lower ones
    if parse_version(new) <= parse_version(current):
        raise ValueError(f"version {new} is not higher than {current}")


def uv_version(project_dir: Path, *args: str) -> str:
    result = subprocess.run(
        ["uv", "version", "--short", *args],
        cwd=project_dir,
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def read_version(project_dir: Path = PROJECT_DIR) -> str:
    # Note: reading leaves the lock file and the environment untouched
    return uv_version(project_dir)


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


# Note: top level keys of CITATION.cff, edited as text to keep its comments and to need no YAML parser
CITATION_KEYS = ("version", "date-released")


def _citation_key_pattern(key: str) -> re.Pattern:
    return re.compile(rf"^{re.escape(key)}:[ \t]*(.*?)[ \t]*$", re.MULTILINE)


def citation_release(citation: str) -> tuple[str, datetime.date]:
    """Return the version and the release date named in CITATION.cff."""
    values = []
    for key in CITATION_KEYS:
        matches = _citation_key_pattern(key).findall(citation)
        if len(matches) != 1:
            raise ValueError(f"{CITATION_NAME} must have exactly one top level '{key}'")
        values.append(matches[0].strip("'\""))
    version, date = values
    try:
        return version, datetime.date.fromisoformat(date)
    except ValueError as error:
        raise ValueError(
            f"invalid date-released {date!r} in {CITATION_NAME}, expected YYYY-MM-DD"
        ) from error


def release_citation(citation: str, version: str, date: datetime.date) -> str:
    """Set the version and the release date in CITATION.cff."""
    citation_release(citation)
    for key, value in zip(CITATION_KEYS, (version, date.isoformat()), strict=True):
        citation = _citation_key_pattern(key).sub(f"{key}: {value}", citation)
    return citation


def check_citation(citation: str, version: str, date: datetime.date) -> None:
    named = citation_release(citation)
    if named != (version, date):
        raise ValueError(
            f"{CITATION_NAME} names version {named[0]} ({named[1].isoformat()}), "
            f"expected {version} ({date.isoformat()})"
        )


def check_release(project_dir: Path = PROJECT_DIR) -> str:
    """checks the changelog section and the citation metadata of the package version

    :return: what was checked
    """
    version = read_version(project_dir)
    changelog = (project_dir / CHANGELOG_NAME).read_text(encoding="utf-8")
    date = release_date(changelog, version)
    citation = (project_dir / CITATION_NAME).read_text(encoding="utf-8")
    check_citation(citation, version, date)
    return f"{CHANGELOG_NAME}, {CITATION_NAME}: {version} ({date.isoformat()})"


def _git(project_dir: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", *args], cwd=project_dir, check=False, capture_output=True, text=True
    )


def _locked_versions(project_dir: Path) -> dict[str, set[str]]:
    path = project_dir / "uv.lock"
    if not path.exists():
        return {}
    with open(path, "rb") as f:
        lock = tomllib.load(f)
    versions = {}
    for package in lock.get("package", []):
        entry = versions.setdefault(package["name"], set())
        # Note: e.g. workspace members with a dynamic version have none
        if package.get("version"):
            entry.add(package["version"])
    return versions


def outdated_benchmarks(project_dir: Path = PROJECT_DIR) -> list[str]:
    """the reasons why the published benchmark results are outdated, empty if they are up to date

    The results are up to date when they come from a complete full run of committed code,
    the code of the package and the benchmark is unchanged since (also uncommitted changes count)
    and the measured dependency versions are the locked ones.
    """
    path = project_dir / BENCHMARK_RESULTS
    if not path.exists():
        return [f"{BENCHMARK_RESULTS} does not exist"]
    with open(path, encoding="utf-8") as f:
        meta = json.load(f)["meta"]
    reasons = []
    if meta.get("quick") or not meta.get("completed"):
        reasons.append("the results are not from a complete full run")
    git = meta.get("git") or {}
    sha = git.get("sha")
    if git.get("dirty"):
        reasons.append(
            f"the results were measured with uncommitted changes (commit {sha})"
        )
    if sha is None:
        reasons.append("the results do not name the measured commit")
    else:
        fetch_error = ""
        if _git(project_dir, "cat-file", "-e", f"{sha}^{{commit}}").returncode != 0:
            # e.g. a shallow clone in CI
            fetch = _git(project_dir, "fetch", "--quiet", "--depth=1", "origin", sha)
            fetch_error = fetch.stderr.strip()
        diff = _git(project_dir, "diff", "--name-only", sha, "--", *BENCHMARKED_CODE)
        # Note: git diff ignores untracked files
        untracked = _git(
            project_dir,
            "ls-files",
            "--others",
            "--exclude-standard",
            "--",
            *BENCHMARKED_CODE,
        )
        if diff.returncode != 0:
            error = "; ".join(e for e in (fetch_error, diff.stderr.strip()) if e)
            reasons.append(
                f"cannot compare with the measured commit {sha[:10]}: {error}"
            )
        elif changed := diff.stdout.split() + untracked.stdout.split():
            reasons.append(
                f"changed since the measured commit {sha[:10]}: {', '.join(changed)}"
            )
    locked = _locked_versions(project_dir)
    for name, measured in sorted((meta.get("versions") or {}).items()):
        if name == "extremitypathfinder":
            # the version of the package itself changes with the release
            continue
        versions = locked.get(name, set())
        if (measured is None and versions) or (
            measured is not None and measured not in versions
        ):
            now = ", ".join(sorted(versions)) or "not in uv.lock"
            reasons.append(
                f"{name}: measured {measured or 'not installed'}, locked {now}"
            )
    return reasons


def version_at(commit: str, project_dir: Path = PROJECT_DIR) -> str:
    """the package version in pyproject.toml at the given commit (fetched in a shallow clone)"""
    fetch_error = ""
    if _git(project_dir, "cat-file", "-e", f"{commit}^{{commit}}").returncode != 0:
        fetch = _git(project_dir, "fetch", "--quiet", "--depth=1", "origin", commit)
        fetch_error = fetch.stderr.strip()
    show = _git(project_dir, "show", f"{commit}:pyproject.toml")
    if show.returncode != 0:
        error = "; ".join(e for e in (fetch_error, show.stderr.strip()) if e)
        raise ValueError(f"cannot read pyproject.toml at {commit}: {error}")
    try:
        return tomllib.loads(show.stdout)["project"]["version"]
    except (tomllib.TOMLDecodeError, KeyError) as error:
        raise ValueError(f"no package version in pyproject.toml at {commit}") from error


def check_release_benchmarks(
    base: str | None = None, project_dir: Path = PROJECT_DIR
) -> str:
    """checks the benchmark results, with a base commit only if the package version changed since

    :return: what was checked
    """
    if base is not None:
        version = read_version(project_dir)
        if version_at(base, project_dir) == version:
            return f"version {version} unchanged since {base}: benchmark results not checked"
    check_benchmarks(project_dir)
    return f"{BENCHMARK_RESULTS}: up to date"


def check_benchmarks(project_dir: Path = PROJECT_DIR) -> None:
    reasons = outdated_benchmarks(project_dir)
    if reasons:
        details = "".join(f"\n  - {reason}" for reason in reasons)
        raise ValueError(f"the benchmark results are outdated:{details}\n{RERUN_HINT}")


def prepare(target: str, date: datetime.date, project_dir: Path = PROJECT_DIR) -> str:
    changelog_path = project_dir / CHANGELOG_NAME
    citation_path = project_dir / CITATION_NAME
    current = read_version(project_dir)
    if target in BUMP_RULES:
        version_args = ("--bump", target)
    else:
        parse_version(target)
        version_args = (target,)
    version = uv_version(project_dir, *version_args, "--dry-run")
    check_new_version(current, version)
    # validate the changelog, the citation metadata and the benchmark results before changing any file
    new_changelog = release_changelog(
        changelog_path.read_text(encoding="utf-8"), version, date
    )
    new_citation = release_citation(
        citation_path.read_text(encoding="utf-8"), version, date
    )
    check_benchmarks(project_dir)
    # Note: also updates the version of the project in the lock file
    uv_version(project_dir, version, "--no-sync")
    changelog_path.write_text(new_changelog, encoding="utf-8")
    citation_path.write_text(new_citation, encoding="utf-8")
    return version


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("version", help="print the package version")
    commands.add_parser(
        "check",
        help="check that the changelog has a dated section for the package version "
        "and that CITATION.cff names the version and its date",
    )
    benchmarks_parser = commands.add_parser(
        "benchmarks", help="check that the published benchmark results are up to date"
    )
    benchmarks_parser.add_argument(
        "--if-version-changed-since",
        metavar="COMMIT",
        help="check only if the package version differs from the version at this commit "
        "(e.g. the base of a pull request)",
    )
    prepare_parser = commands.add_parser(
        "prepare",
        help="set the new version, date the 'Unreleased' changelog section "
        "and update CITATION.cff",
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

    try:
        if args.command == "version":
            print(read_version())
        elif args.command == "check":
            print(check_release())
        elif args.command == "benchmarks":
            print(check_release_benchmarks(args.if_version_changed_since))
        else:
            print(prepare(args.target, args.date))
    except ValueError as error:
        parser.exit(1, f"error: {error}\n")
    except subprocess.CalledProcessError as error:
        parser.exit(1, f"error: uv version failed:\n{error.stderr}")
    except FileNotFoundError:
        parser.exit(1, "error: uv and git are required\n")


if __name__ == "__main__":
    main()
