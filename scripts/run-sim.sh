#!/usr/bin/env bash
# Runs the core simulation on a topology and writes a run directory.
#
# usage: scripts/run-sim.sh <topology> [--view] [cli options]
#   <topology>   a name from topologies/ (e.g. planar_x) or a path to a .json file
#   --view       open the run in the Godot viewer when it finishes
#   cli options  passed to `tensegrity run`: --seed N, --dt S, --duration S,
#                --frame-stride N, --out DIR
#
# Examples:
#   scripts/run-sim.sh planar_x
#   scripts/run-sim.sh cable_oscillator --duration 5 --view
#
# Without --out, the run goes to runs/<date>_<time>_<topology>, so
# repeated runs never collide. With no arguments, lists the topologies.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

list_topologies() {
    echo "Topologies in topologies/:" >&2
    for file in "$root"/topologies/*.json; do
        echo "  $(basename "$file" .json)" >&2
    done
}

if [[ $# -eq 0 || "$1" == -h || "$1" == --help ]]; then
    sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
    list_topologies
    exit 2
fi

topology="$1"
shift
if [[ -f "$topology" ]]; then
    topology="$(cd "$(dirname "$topology")" && pwd)/$(basename "$topology")"
elif [[ -f "$root/topologies/$topology.json" ]]; then
    topology="$root/topologies/$topology.json"
else
    echo "No topology '$topology'." >&2
    list_topologies
    exit 2
fi

view=false
has_out=false
args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --view) view=true ;;
        --out)
            # The CLI runs from the repository root; keep a relative --out
            # relative to where this script was called from.
            has_out=true
            if [[ $# -ge 2 ]]; then
                out="$2"
                [[ "$out" = /* ]] || out="$PWD/$out"
                args+=(--out "$out")
                shift
            else
                args+=(--out)
            fi
            ;;
        *) args+=("$1") ;;
    esac
    shift
done

if [[ "$has_out" == false ]]; then
    args+=(--out "$root/runs/$(date +%Y-%m-%d_%H%M%S)_$(basename "$topology" .json)")
fi

# Exit code 3 means the run was written but a strut went into tension:
# report it, still offer the run to the viewer, and pass the code on.
status=0
(cd "$root" && dotnet run --project src/Tensegrity.Cli -- run --topology "$topology" "${args[@]}") || status=$?

if [[ "$view" == true && ( $status -eq 0 || $status -eq 3 ) ]]; then
    "$root/scripts/view-latest.sh"
fi

exit "$status"
