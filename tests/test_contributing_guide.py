from pathlib import Path

import pytest

from scripts.check_contributing_guide import check, count_words

CONFIG = """[tool.contributing-guide]
root = "docs/contributing"
max-words = 10
referrers = ["CONTRIBUTING.md", "docs/**/*.md", "docs/**/*.rst", ".claude/**/*.md"]
"""
UNREFERENCED = ": not referenced by any other file (CONTRIBUTING.md, AGENTS.md, a page of the documentation or a skill)"


def make_project(
    tmp_path: Path,
    pages: dict[str, str],
    contributing: str = "",
    extra_config: str = "",
) -> Path:
    (tmp_path / "pyproject.toml").write_text(CONFIG + extra_config)
    (tmp_path / "CONTRIBUTING.md").write_text(contributing)
    for name, text in pages.items():
        path = tmp_path / "docs" / "contributing" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    return tmp_path


def errors_of(project: Path) -> list[str]:
    return check(project)[0]


def test_the_contributing_guide_meets_its_rules():
    errors, counts, _ = check()
    assert counts, "no contributing pages found"
    assert errors == []


def test_count_words():
    # like `wc -w`: list markers and link targets count
    assert count_words("one two\n\n- three `four`  [five](six.md)\n") == 6


def test_valid_project(tmp_path):
    project = make_project(
        tmp_path,
        {"a.md": "see [b](b.md)", "b.md": "short", "sub/c.md": "see [a](../a.md)"},
        contributing="[A](docs/contributing/a.md) and `docs/contributing/sub/c.md`",
    )
    assert errors_of(project) == []


def test_page_over_the_limit(tmp_path):
    project = make_project(
        tmp_path, {"a.md": "word " * 11}, contributing="docs/contributing/a.md"
    )
    (error,) = errors_of(project)
    assert error.startswith("a.md: 11 words, limit 10")


def test_exception_raises_the_limit(tmp_path):
    exception = '[tool.contributing-guide.exceptions."a.md"]\nmax-words = 12\nreason = "one table"\n'
    project = make_project(
        tmp_path, {"a.md": "word " * 11}, "docs/contributing/a.md", exception
    )
    assert errors_of(project) == []


@pytest.mark.parametrize(
    "exception, words, message",
    [
        ('max-words = 12\nreason = ""', 11, "a reason (text) is required"),
        (
            'max-words = 10\nreason = "x"',
            11,
            "must be a number higher than the default",
        ),
        ('max-words = 12\nreason = "x"', 9, "Remove its exception"),
        ('max-words = 12\nreason = "x"', 13, "13 words, limit 12"),
        ("max-words = 12\nreason = 1", 11, "a reason (text) is required"),
        ('reason = "x"', 11, "must be a number higher than the default"),
        (
            'max-words = "12"\nreason = "x"',
            11,
            "must be a number higher than the default",
        ),
    ],
)
def test_invalid_or_violated_exceptions(tmp_path, exception, words, message):
    config = f'[tool.contributing-guide.exceptions."a.md"]\n{exception}\n'
    project = make_project(
        tmp_path, {"a.md": "word " * words}, "docs/contributing/a.md", config
    )
    errors = errors_of(project)
    assert any(message in error for error in errors), errors


def test_exception_for_a_missing_page(tmp_path):
    config = (
        '[tool.contributing-guide.exceptions."gone.md"]\nmax-words = 12\nreason = "x"\n'
    )
    project = make_project(tmp_path, {"a.md": "a"}, "docs/contributing/a.md", config)
    assert errors_of(project) == [
        "exception for gone.md: no such page under contributing/"
    ]


def test_unreferenced_files(tmp_path):
    project = make_project(
        tmp_path, {"a.md": "see [a](a.md)", "sub/b.md": "", "fig.svg": ""}
    )
    assert errors_of(project) == [
        f"a.md{UNREFERENCED}",
        f"fig.svg{UNREFERENCED}",
        f"sub/b.md{UNREFERENCED}",
    ]


def test_toctree_entries_do_not_count_but_doc_roles_do(tmp_path):
    project = make_project(tmp_path, {"a.md": "", "b.md": ""})
    (tmp_path / "docs" / "index.md").write_text(
        "```{toctree}\ncontributing/a\ncontributing/b\n```\n"
    )
    (tmp_path / "docs" / "design.rst").write_text("the :doc:`log <contributing/b>`\n")
    assert errors_of(project) == [f"a.md{UNREFERENCED}"]


def test_references_must_resolve(tmp_path):
    # the same file name in another directory is not a reference
    project = make_project(tmp_path, {"a.md": ""}, contributing="[a](docs/a.md)")
    assert errors_of(project) == [f"a.md{UNREFERENCED}"]


def test_rst_pages_have_the_limit_too(tmp_path):
    project = make_project(tmp_path, {"a.rst": "word " * 11}, "docs/contributing/a.rst")
    (error,) = errors_of(project)
    assert error.startswith("a.rst: 11 words, limit 10")


def test_skills_are_referrers(tmp_path):
    project = make_project(tmp_path, {"a.md": ""})
    skill = tmp_path / ".claude" / "skills" / "x" / "SKILL.md"
    skill.parent.mkdir(parents=True)
    skill.write_text("[a](../../../docs/contributing/a.md)")
    assert errors_of(project) == []


def test_project_absolute_paths_are_references(tmp_path):
    project = make_project(
        tmp_path, {"a.md": ""}, contributing="[a](/docs/contributing/a.md)"
    )
    assert errors_of(project) == []


def test_hidden_files_and_build_directories_are_skipped(tmp_path):
    project = make_project(
        tmp_path,
        {
            "a.md": "",
            ".DS_Store": "",
            ".cache/b.md": "word " * 11,
            "_build/c.md": "word " * 11,
        },
        contributing="docs/contributing/a.md",
    )
    # a built copy does not count as a reference
    (tmp_path / "docs" / "_build").mkdir()
    (tmp_path / "docs" / "_build" / "d.md").write_text("contributing/e.md")
    (tmp_path / "docs" / "contributing" / "e.md").write_text("")
    assert errors_of(project) == [f"e.md{UNREFERENCED}"]


def test_urls_are_no_references(tmp_path):
    url = "https://github.com/owner/repo/blob/master/docs/contributing/a.md"
    project = make_project(tmp_path, {"a.md": ""}, contributing=f"[a]({url})")
    assert errors_of(project) == [f"a.md{UNREFERENCED}"]
