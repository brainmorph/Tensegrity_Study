# Runs the core simulation on a topology and writes a run directory.
# Windows PowerShell version of run-sim.sh, with the same arguments.
#
# usage: scripts\run-sim.ps1 <topology> [--view] [cli options]
#   <topology>   a name from topologies\ (e.g. planar_x) or a path to a .json file
#   --view       open the run in the Godot viewer when it finishes
#   cli options  passed to `tensegrity run`: --seed N, --dt S, --duration S,
#                --frame-stride N, --out DIR, and the perturbation flags
#
# Examples:
#   scripts\run-sim.ps1 planar_x
#   scripts\run-sim.ps1 toggle_arm_snap --view
#   scripts\run-sim.ps1 cable_oscillator --duration 5 --view
#
# Without --out, the run goes to runs\<date>_<time>_<topology>, so
# repeated runs never collide. With no arguments, lists the topologies.
#
# There is deliberately no param() block: PowerShell would read --view and
# --duration as its own parameters. Every argument arrives in $args as text.

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

function Show-Topologies {
    [Console]::Error.WriteLine('Topologies in topologies\:')
    Get-ChildItem (Join-Path $root 'topologies') -Filter '*.json' | Sort-Object Name | ForEach-Object {
        [Console]::Error.WriteLine("  $($_.BaseName)")
    }
}

function Show-Usage {
    # The comment block at the top of this file, without its '# ' prefix.
    Get-Content $PSCommandPath | Select-Object -First 16 | ForEach-Object {
        [Console]::Error.WriteLine(($_ -replace '^# ?', ''))
    }
}

$arguments = @($args | ForEach-Object { [string]$_ })
if ($arguments.Count -eq 0 -or $arguments[0] -in '-h', '--help', '/?') {
    Show-Usage
    Show-Topologies
    exit 2
}

$topology = $arguments[0]
$named = Join-Path $root "topologies\$topology.json"
if (Test-Path -LiteralPath $topology -PathType Leaf) {
    $topology = (Resolve-Path -LiteralPath $topology).Path
}
elseif (Test-Path -LiteralPath $named -PathType Leaf) {
    $topology = $named
}
else {
    [Console]::Error.WriteLine("No topology '$topology'.")
    Show-Topologies
    exit 2
}

$view = $false
$hasOut = $false
$cliArgs = New-Object System.Collections.Generic.List[string]
for ($i = 1; $i -lt $arguments.Count; $i++) {
    switch ($arguments[$i]) {
        '--view' { $view = $true }
        '--out' {
            # The CLI runs from the repository root; keep a relative --out
            # relative to where this script was called from.
            $hasOut = $true
            $cliArgs.Add('--out')
            if ($i + 1 -lt $arguments.Count) {
                $i++
                $out = $arguments[$i]
                if (-not [IO.Path]::IsPathRooted($out)) {
                    $out = Join-Path (Get-Location).Path $out
                }
                $cliArgs.Add($out)
            }
        }
        default { $cliArgs.Add($arguments[$i]) }
    }
}

if (-not $hasOut) {
    $stamp = Get-Date -Format 'yyyy-MM-dd_HHmmss'
    $name = [IO.Path]::GetFileNameWithoutExtension($topology)
    $cliArgs.Add('--out')
    $cliArgs.Add((Join-Path $root "runs\${stamp}_$name"))
}

# Exit code 3 means the run was written but a strut went into tension:
# report it, still offer the run to the viewer, and pass the code on.
Push-Location $root
try {
    & dotnet run --project src/Tensegrity.Cli -- run --topology $topology @cliArgs
    $status = $LASTEXITCODE
}
finally {
    Pop-Location
}

if ($view -and ($status -eq 0 -or $status -eq 3)) {
    & (Join-Path $PSScriptRoot 'view-latest.ps1')
}

exit $status
