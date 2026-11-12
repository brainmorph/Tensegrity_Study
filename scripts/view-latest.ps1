# Opens the most recent run under runs\ in the Godot viewer.
# Windows PowerShell version of view-latest.sh, with the same arguments.
#
# usage: scripts\view-latest.ps1 [viewer options]
#   e.g. scripts\view-latest.ps1 --play
#
# "Most recent" is the run directory whose manifest.json was written
# last. The viewer is rebuilt first (a no-op when up to date). The Godot
# executable comes from TENSEGRITY_GODOT, else the Windows install path in
# PLAN.md section 3.
#
# No param() block, so viewer options such as --play pass through as text.

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$runs = Join-Path $root 'runs'

$godot = $env:TENSEGRITY_GODOT
if (-not $godot) {
    $godot = Join-Path $env:USERPROFILE 'Downloads\Godot_v4.7-stable_mono_win64\Godot_v4.7-stable_mono_win64\Godot_v4.7-stable_mono_win64.exe'
}

if (-not (Test-Path -LiteralPath $godot -PathType Leaf)) {
    [Console]::Error.WriteLine("Godot not found at $godot; set TENSEGRITY_GODOT to its path.")
    exit 1
}

$latest = $null
if (Test-Path -LiteralPath $runs -PathType Container) {
    $latest = Get-ChildItem -LiteralPath $runs -Filter 'manifest.json' -File -Recurse |
        Sort-Object LastWriteTimeUtc |
        Select-Object -Last 1 |
        ForEach-Object { $_.DirectoryName }
}

if (-not $latest) {
    [Console]::Error.WriteLine("No runs found under $runs. Record one first, for example:")
    [Console]::Error.WriteLine('  scripts\run-sim.ps1 planar_x')
    exit 1
}

Write-Host "Opening $($latest.Substring($root.Length + 1))"
& dotnet build (Join-Path $root 'godot\TensegrityViewer.csproj') -nologo -v q | Out-Null
if ($LASTEXITCODE -ne 0) {
    [Console]::Error.WriteLine('The viewer did not build; run: dotnet build godot\TensegrityViewer.csproj')
    exit $LASTEXITCODE
}

# Start Godot detached, as a normal window: run directly, it is tied to
# the calling console and closes with it. Paths are quoted because
# Start-Process joins the arguments into one command line.
$viewerArgs = @('--path', "`"$(Join-Path $root 'godot')`"", '--', '--run', "`"$latest`"") + @($args | ForEach-Object { [string]$_ })
Start-Process -FilePath $godot -ArgumentList $viewerArgs
