#!/usr/bin/env bash
# Obtain the speech executable locally; does not install or change system packages.
set -euo pipefail
cd "$(dirname "$0")/.."
if command -v espeak-ng >/dev/null; then exit 0; fi
mkdir -p .tools
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
(cd "$stage" && apt download espeak-ng && dpkg-deb -x espeak-ng*.deb extracted)
cp "$stage/extracted/usr/bin/espeak-ng" .tools/espeak-ng
.tools/espeak-ng --version
echo 'Uses the host espeak-ng shared library/data; tested on this Ubuntu laptop.'
