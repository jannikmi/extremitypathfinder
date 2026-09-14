from pathlib import Path

import pytest

from scripts.validate_release import find_artifacts, virtualenv_executable


def test_find_artifacts(tmp_path):
    wheel = tmp_path / "package.whl"
    source_distribution = tmp_path / "package.tar.gz"
    wheel.touch()
    source_distribution.touch()

    assert find_artifacts(tmp_path) == (wheel, source_distribution)


def test_find_artifacts_requires_one_of_each(tmp_path):
    (tmp_path / "package.whl").touch()

    with pytest.raises(ValueError, match="one wheel and one .tar.gz"):
        find_artifacts(tmp_path)


def test_virtualenv_executable_uses_platform_layout(monkeypatch):
    monkeypatch.setattr("scripts.validate_release.sys.platform", "win32")

    assert virtualenv_executable(Path("environment"), "python") == Path(
        "environment/Scripts/python.exe"
    )
