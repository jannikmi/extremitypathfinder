# Releasing

Releases are done by the maintainers. Contributors and agents never bump the version, push tags
or change the release workflows unless they are asked to.

## Release validation

Before publishing a release candidate, build and validate the exact artifacts users will install:

```sh
make release-check
```

This command cleans `dist/`, builds one wheel and one source distribution, and installs each in
an isolated virtual environment outside the source checkout. It smoke-tests package metadata, the
public API, the README example, the console command, dependency consistency, and the wheel with the
optional Numba extra. The same validation gates pull requests and runs again on the artifacts built
immediately before publishing.

The release tooling (`scripts/release.py`) is tested in a separate tox environment, which also runs in CI:
`uv run tox -e release`.

## Releasing

Releases are automated with GitHub Actions:

1. Run the "prepare release" workflow from the Actions tab and enter `patch`, `minor`, `major`
   or an explicit version. It sets the new version with `uv version`, turns the `Unreleased`
   changelog section into the section of the new version dated today, and pushes the branch
   `release/<version>`. The job summary links to the pull request to open.
   Locally the same is done by `make release-prepare BUMP=patch`.
2. Open the pull request, let CI pass and merge it.
3. The "tag release" workflow notices the new version on `master`: it checks the changelog,
   pushes a tag exactly matching the version, and starts the build workflow on that tag.
4. The build workflow tests the tag and validates the release artifacts again. Its deploy job verifies
   that the version is not already on PyPI, uploads the validated artifacts to a draft GitHub release,
   publishes them to PyPI, and makes the GitHub release public only after the PyPI upload succeeds.

If a step fails after the tag exists, fix the cause and re-run the build workflow on the tag
(Actions tab: "build", "Run workflow", select the tag).
Pushing a tag manually still triggers the build workflow as well.
