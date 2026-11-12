# Tensegrity viewer (Godot 4.7 mono)

Plays back runs recorded by the core. It reads a run directory
(`manifest.json` + `frames.bin`) through `Tensegrity.RunFormat` and
draws exactly what was recorded. It runs no physics and changes nothing
on disk (PLAN.md section 1a, constraint 5).

## Build and open a run

```sh
# from the repository root
dotnet run --project src/Tensegrity.Cli -- run --topology topologies/planar_x.json --out runs/planar_x
dotnet build godot/TensegrityViewer.csproj

GODOT=~/Desktop/Godot_v4.7-stable_mono_linux_x86_64/Godot_v4.7-stable_mono_linux.x86_64
$GODOT --path godot -- --run runs/planar_x          # open a run
$GODOT --path godot -- --run runs/planar_x --play   # open and start playing
$GODOT --path godot                                 # pick a run directory in the app
```

Or let `scripts/view-latest.sh` build the viewer and open the most
recently written run under `runs/`, passing any viewer options through:

```sh
scripts/view-latest.sh --play
```

On Windows, use the PowerShell versions, which take the same arguments.
`scripts\run-sim.ps1` records a run (a topology name or a .json path)
and `--view` opens it when it finishes. `scripts\view-latest.ps1` opens
the newest run. The `.cmd` files beside them run the same scripts from
cmd.exe or a double-click, whatever the execution policy. Godot comes
from `TENSEGRITY_GODOT`, else the path in PLAN.md section 3:

```powershell
scripts\run-sim.ps1 toggle_arm_snap --view
scripts\view-latest.ps1 --play
```

Relative `--run` paths resolve against the directory you launch from.
You can also open the project in the Godot editor and run it from there.
The folder picker then starts in `runs/`.

## What is drawn

| Element | Drawn from |
|---|---|
| Link | Bar of the topology's `length`, centered at the recorded `link.<id>.x`, `.y` and rotated by `.theta`. Struts are a little wider and warmer in colour. |
| Cable | Line between the recorded endpoints `cable.<id>.ax`, `.ay`, `.bx`, `.by`. Its colour follows the recorded `cable.<id>.tension` on one scale per run, from 0 N to the highest tension in the manifest's `diagnostics.cable_tensions`. A slack cable (tension 0) is grey and dashed. |
| Pin | Dot at the recorded `pin.<id>.x`, `.y`. Pins to the ground get a base. |

Frames are shown exactly as recorded. During playback the frame on show
is the last one recorded at or before the playback clock. Nothing is
interpolated. The world node's only transform is a y-flip: the run is
y-up and the screen is y-down.

The info panel shows recorded manifest facts only: frame count, step,
energy-balance error, pin residual, event count, and warnings for struts
in tension or unresolved events.

## Controls

| Input | Action |
|---|---|
| Space | Play / pause |
| ← / → | Step one frame |
| Home / End | First / last frame |
| Drag (any mouse button) | Pan |
| Wheel | Zoom about the cursor |
| F | Fit the run to the window |
| O | Open another run |

The speed menu sets run seconds per real second.

## Guarantees and checks

- The project's only reference is `Tensegrity.RunFormat`. Both physics
  engines are set to Godot's `Dummy` server.
- `tests/Tensegrity.Core.Tests/Visualizer` scans this folder on every
  `dotnet test`. It fails on a reference to a solver project, on any
  physics node or physics-server call, and on any file-writing API.
