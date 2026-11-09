#!/usr/bin/env bash
# Opens the most recent run under runs/ in the Godot viewer.
#
# usage: scripts/view-latest.sh [viewer options]
#   e.g. scripts/view-latest.sh --play
#
# "Most recent" is the run directory whose manifest.json was written
# last. The viewer is rebuilt first (a no-op when up to date). The Godot
# executable comes from TENSEGRITY_GODOT, else the install path in
# PLAN.md section 3.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runs="$root/runs"
godot="${TENSEGRITY_GODOT:-$HOME/Desktop/Godot_v4.7-stable_mono_linux_x86_64/Godot_v4.7-stable_mono_linux.x86_64}"

if [[ ! -x "$godot" ]]; then
    echo "Godot not found at $godot; set TENSEGRITY_GODOT to its path." >&2
    exit 1
fi

latest=""
if [[ -d "$runs" ]]; then
    latest="$(find "$runs" -name manifest.json -type f -printf '%T@ %h\n' | sort -n | tail -n 1 | cut -d' ' -f2-)"
fi

if [[ -z "$latest" ]]; then
    echo "No runs found under $runs. Record one first, for example:" >&2
    echo "  dotnet run --project src/Tensegrity.Cli -- run --topology topologies/planar_x.json --out runs/planar_x" >&2
    exit 1
fi

echo "Opening ${latest#"$root"/}"
dotnet build "$root/godot/TensegrityViewer.csproj" -nologo -v q >/dev/null
exec "$godot" --path "$root/godot" -- --run "$latest" "$@"
