"""Validate built release artifacts in isolated virtual environments."""

import argparse
import json
import subprocess
import sys
import tempfile
import venv
from pathlib import Path


SMOKE_TEST = """
import json
from importlib.metadata import version
from pathlib import Path

from extremitypathfinder import PolygonEnvironment

assert version("extremitypathfinder") == {version!r}

environment = PolygonEnvironment()
boundary_coordinates = [
    (0.0, 0.0),
    (10.0, 0.0),
    (9.0, 5.0),
    (10.0, 10.0),
    (0.0, 10.0),
]
list_of_holes = [
    [(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0)],
]
environment.store(boundary_coordinates, list_of_holes, validate=True)
path, length = environment.find_shortest_path((4.5, 1.0), (4.0, 8.5))
assert path[0] == (4.5, 1.0)
assert path[-1] == (4.0, 8.5)
assert length > 0.0

Path("smoke-result.json").write_text(
    json.dumps({{"path": path, "length": length}}), encoding="utf-8"
)
"""

NUMBA_TEST = """
from extremitypathfinder import utils_numba

assert hasattr(utils_numba._lies_behind_inner, "nopython_signatures")
assert utils_numba._lies_behind_inner.nopython_signatures
"""


def run(
    command: list[str], *, cwd: Path, capture_output: bool = False
) -> subprocess.CompletedProcess[str]:
    """Run a checked command while keeping its output visible in CI logs."""
    print("+", " ".join(str(part) for part in command), flush=True)
    result = subprocess.run(
        command,
        cwd=cwd,
        check=True,
        text=True,
        stdout=subprocess.PIPE if capture_output else None,
        stderr=subprocess.STDOUT if capture_output else None,
    )
    if capture_output:
        print(result.stdout, end="", flush=True)
    return result


def virtualenv_executable(environment: Path, executable: str) -> Path:
    """Return an executable path for POSIX and Windows virtual environments."""
    scripts_directory = "Scripts" if sys.platform == "win32" else "bin"
    suffix = ".exe" if sys.platform == "win32" else ""
    return environment / scripts_directory / f"{executable}{suffix}"


def create_virtualenv(environment: Path) -> None:
    """Create an isolated environment without copying signed POSIX interpreters."""
    venv.EnvBuilder(
        with_pip=True,
        clear=True,
        symlinks=sys.platform != "win32",
    ).create(environment)


def validate_install(
    artifact: Path,
    *,
    version: str,
    example: Path,
    install_numba: bool,
) -> None:
    """Install and smoke-test one artifact outside the source checkout."""
    label = f"{artifact.name}{'[numba]' if install_numba else ''}"
    print(f"\nValidating {label}", flush=True)
    with tempfile.TemporaryDirectory(
        prefix="extremitypathfinder-release-"
    ) as directory:
        workspace = Path(directory)
        environment = workspace / "venv"
        create_virtualenv(environment)
        python = virtualenv_executable(environment, "python")
        pip = [str(python), "-m", "pip"]
        requirement = f"{artifact}[numba]" if install_numba else str(artifact)

        run(
            [*pip, "install", "--disable-pip-version-check", requirement], cwd=workspace
        )
        run([*pip, "check"], cwd=workspace)
        run(
            [str(python), "-I", "-c", SMOKE_TEST.format(version=version)],
            cwd=workspace,
        )

        cli = virtualenv_executable(environment, "extremitypathfinder")
        result = run(
            [
                str(cli),
                str(example),
                "--start",
                "2.5",
                "3.2",
                "--goal",
                "7.9",
                "6.8",
            ],
            cwd=workspace,
            capture_output=True,
        )
        if not result.stdout:
            raise RuntimeError("CLI smoke test did not produce a path")

        if install_numba:
            run([str(python), "-I", "-c", NUMBA_TEST], cwd=workspace)


def find_artifacts(distribution_directory: Path) -> tuple[Path, Path]:
    """Find exactly one wheel and source distribution."""
    wheels = list(distribution_directory.glob("*.whl"))
    source_distributions = list(distribution_directory.glob("*.tar.gz"))
    if len(wheels) != 1 or len(source_distributions) != 1:
        raise ValueError(
            "Expected exactly one wheel and one .tar.gz source distribution in "
            f"{distribution_directory}; found {len(wheels)} wheel(s) and "
            f"{len(source_distributions)} source distribution(s)."
        )
    return wheels[0].resolve(), source_distributions[0].resolve()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("distribution_directory", type=Path)
    parser.add_argument("--version", required=True)
    parser.add_argument(
        "--example",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "example.json",
    )
    args = parser.parse_args()

    wheel, source_distribution = find_artifacts(args.distribution_directory)
    example = args.example.resolve()
    for artifact in (wheel, source_distribution):
        validate_install(
            artifact,
            version=args.version,
            example=example,
            install_numba=False,
        )
    validate_install(
        wheel,
        version=args.version,
        example=example,
        install_numba=True,
    )

    print(
        "\nRelease validation passed:\n"
        + json.dumps(
            {
                "version": args.version,
                "wheel": wheel.name,
                "source_distribution": source_distribution.name,
                "numba_extra": "wheel",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
