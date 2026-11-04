"""Check the size and the references of the contributing guide.

usage:
    python scripts/check_contributing_guide.py           # check, print the violations
    python scripts/check_contributing_guide.py --list    # also print the word count of every page

Rules (cf. ``docs/contributing/documentation.md``):

- every page (Markdown or reStructuredText) under the guide's root has at most ``max-words`` words,
  unless an exception with a higher limit and a reason is configured for it.
  An exception is only allowed while the page exceeds the default limit.
- every file under the guide's root is referenced by at least one other file of ``referrers``
  (a relative or project-relative path, or a ``:doc:`` / ``{doc}`` role).
  The hidden toctree in ``docs/5_contributing.md`` does not count: its entries have no file extension
  and are no ``doc`` roles, it only makes the page part of the build.

Hidden files (e.g. ``.DS_Store``) and files in hidden or build directories of the guide are skipped.

The configuration is the ``[tool.contributing-guide]`` table in ``pyproject.toml``.
Words are counted like ``wc -w``: sequences of non-whitespace characters, including code and link targets.
"""

import argparse
import os
import re
import sys
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_DIR = Path(__file__).parent.parent
CONFIG_TABLE = "contributing-guide"
IGNORED_DIR_NAMES = {"_build", "__pycache__"}
PAGE_SUFFIXES = (".md", ".rst")

PATH_TOKEN = re.compile(r"[\w./-]+")
DOC_ROLE = re.compile(r"[:{]doc[:}]`(?:[^`<]*<)?([^`<>]+)>?`")


@dataclass
class PageException:
    # as configured, validated by check_sizes()
    max_words: object
    reason: object


@dataclass
class Config:
    root: Path
    max_words: int
    referrers: list[str]
    exceptions: dict[str, PageException] = field(default_factory=dict)


def load_config(project_dir: Path) -> Config:
    with open(project_dir / "pyproject.toml", "rb") as file:
        table = tomllib.load(file)["tool"][CONFIG_TABLE]
    exceptions = {
        name: PageException(
            max_words=entry.get("max-words"), reason=entry.get("reason")
        )
        for name, entry in table.get("exceptions", {}).items()
    }
    return Config(
        root=project_dir / table["root"],
        max_words=table["max-words"],
        referrers=table["referrers"],
        exceptions=exceptions,
    )


def count_words(text: str) -> int:
    return len(text.split())


def _in_ignored_dir(path: Path, base: Path) -> bool:
    return any(part in IGNORED_DIR_NAMES for part in path.relative_to(base).parts[:-1])


def guide_files(root: Path) -> list[Path]:
    return sorted(
        p
        for p in root.rglob("*")
        if p.is_file()
        and not _in_ignored_dir(p, root)
        and not any(part.startswith(".") for part in p.relative_to(root).parts)
    )


def referrer_files(project_dir: Path, patterns: list[str]) -> list[Path]:
    files = set()
    for pattern in patterns:
        files.update(
            p
            for p in project_dir.glob(pattern)
            if p.is_file() and not _in_ignored_dir(p, project_dir)
        )
    return sorted(files)


def referenced_paths(referrer: Path, project_dir: Path) -> set[Path]:
    """All existing files a referrer points to, as resolved absolute paths (candidates, not validated links)."""
    text = referrer.read_text(encoding="utf-8")
    candidates = []
    for token in PATH_TOKEN.findall(text):
        token = token.rstrip(".")
        if "." in Path(token).name:
            candidates += [referrer.parent / token, project_dir / token.lstrip("/")]
    for target in DOC_ROLE.findall(text):
        target = target.strip()
        base = (
            project_dir / "docs" / target.lstrip("/")
            if target.startswith("/")
            else referrer.parent / target
        )
        candidates += [base.with_name(base.name + suffix) for suffix in (".md", ".rst")]
    return {Path(os.path.normpath(c)) for c in candidates if c.is_file()}


def check_sizes(config: Config, files: list[Path]) -> tuple[list[str], dict[Path, int]]:
    errors = []
    counts = {}
    pages = {
        p.relative_to(config.root).as_posix(): p
        for p in files
        if p.suffix in PAGE_SUFFIXES
    }
    valid = {}
    for name, exception in config.exceptions.items():
        if name not in pages:
            errors.append(
                f"exception for {name}: no such page under {config.root.name}/"
            )
        elif not isinstance(exception.reason, str) or not exception.reason.strip():
            errors.append(f"exception for {name}: a reason (text) is required")
        elif (
            not isinstance(exception.max_words, int)
            or exception.max_words <= config.max_words
        ):
            errors.append(
                f"exception for {name}: max-words must be a number higher than the default {config.max_words}"
            )
        else:
            valid[name] = exception
    for name, page in pages.items():
        words = count_words(page.read_text(encoding="utf-8"))
        counts[page] = words
        exception = valid.get(name)
        limit = config.max_words if exception is None else exception.max_words
        if words > limit:
            errors.append(
                f"{name}: {words} words, limit {limit}. Condense it or split it by topic "
                "(an exception needs a reason in pyproject.toml)"
            )
        elif exception is not None and words <= config.max_words:
            errors.append(
                f"{name}: {words} words, within the default limit {config.max_words}. Remove its exception"
            )
    return errors, counts


def check_references(config: Config, files: list[Path], project_dir: Path) -> list[str]:
    unreferenced = {Path(os.path.normpath(p.absolute())) for p in files}
    for referrer in referrer_files(project_dir, config.referrers):
        own = Path(os.path.normpath(referrer.absolute()))
        unreferenced -= referenced_paths(
            referrer.absolute(), project_dir.absolute()
        ) - {own}
    return [
        f"{p.relative_to(config.root.absolute()).as_posix()}: not referenced by any other file "
        "(CONTRIBUTING.md, AGENTS.md, a page of the documentation or a skill)"
        for p in sorted(unreferenced)
    ]


def check(project_dir: Path = PROJECT_DIR) -> tuple[list[str], dict[Path, int], Config]:
    project_dir = Path(os.path.normpath(project_dir.absolute()))
    config = load_config(project_dir)
    files = guide_files(config.root)
    errors, counts = check_sizes(config, files)
    errors += check_references(config, files, project_dir)
    return errors, counts, config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--list", action="store_true", help="print the word count of every page"
    )
    args = parser.parse_args(argv)
    errors, counts, config = check()
    if args.list:
        for page, words in sorted(counts.items(), key=lambda item: -item[1]):
            print(f"{words:6d}  {page.relative_to(config.root).as_posix()}")
    for error in errors:
        print(f"contributing guide: {error}", file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
