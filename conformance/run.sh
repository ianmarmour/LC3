#!/bin/sh
# Runs the LC3 conformance script on the `lc3` command.
#
# Usage: conformance/run.sh CONFIG [WORKDIR]
#
# CONFIG is a configuration file of this directory, e.g. lc3_decoder_10ms.cfg.
# The conformance software of third_party/ is extracted to WORKDIR (default:
# target/conformance) with the reference executable, the `lc3` command and
# the configuration. See README.md for the requirements.
set -e

ROOT=$(cd "$(dirname "$0")/.." && pwd)
CONFIG=$1
WORK=${2:-$ROOT/target/conformance}
[ -n "$CONFIG" ] || { sed -n '2,10p' "$0"; exit 1; }

PACKAGE=$(ls "$ROOT"/third_party/LC3_conformance_interoperability_test_software_*.zip | head -n 1)
mkdir -p "$WORK"
if [ ! -f "$WORK/conformanceCheck.py" ]; then
    TMP=$(mktemp -d)
    unzip -q "$PACKAGE" -d "$TMP"
    cp "$TMP"/LC3_Conformance_Interoperability_Script/* "$WORK"/
    mkdir -p "$WORK/LC3_bin_current"
    cp "$TMP"/LC3_Reference_Binary/LC3.exe "$WORK/LC3_bin_current/"
    rm -rf "$TMP"
fi

# The script builds its RMS tool with `gcc -lm rms.c -o rms`, which fails to
# link with toolchains defaulting to --as-needed: put the library last.
mkdir -p "$WORK/shim"
printf '#!/bin/sh\nexec /usr/bin/gcc "$@" -lm\n' > "$WORK/shim/gcc"
chmod +x "$WORK/shim/gcc"

cargo build --release --features cli --manifest-path "$ROOT/Cargo.toml"
cp "$ROOT/target/release/lc3" "$WORK/lc3"
cp "$ROOT/conformance/$CONFIG" "$WORK/"

cd "$WORK"
PATH="$WORK/shim:$PATH" WINEDEBUG=-all python3 conformanceCheck.py -system_sox "$CONFIG"
