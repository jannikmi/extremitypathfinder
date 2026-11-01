# Changelog

`CHANGELOG.rst` lists changes only, newest release first.

- Add entries for changes that are merged but not yet released to the `Unreleased` section at the top.
  When releasing, it becomes the `<version> (<release date>)` section (cf. [Releasing](releasing.md)).
- Describe user-facing changes at the top of a release section, from the user's point of view.
  Start breaking changes with `BREAKING:` and say how to migrate.
- Smaller documentation improvements and cosmetic changes get no entry.
- List bugfixes under a `bugfixes:` sub-list and purely internal changes (tests, tooling, documentation of the
  development) under an `internal:` sub-list.
- CI checks that the changelog has a dated section for the version in `pyproject.toml`.
- Ideas, TODOs and planned changes do not belong in the changelog: track them as
  [GitHub Issues](https://github.com/jannikmi/extremitypathfinder/issues).
