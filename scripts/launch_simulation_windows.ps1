param([switch]$Recompute)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$reportFile = Join-Path $projectRoot 'examples/realtime_20261103/simulation.html'
if ($Recompute) {
    Push-Location $projectRoot
    try {
        $pythonPath = Join-Path $projectRoot '.venv-pc/Scripts/python.exe'
        if (-not (Test-Path -LiteralPath $pythonPath)) {
            python -m venv .venv-pc
            if ($LASTEXITCODE -ne 0) { throw 'Python environment creation failed' }
            & $pythonPath -m pip install -e '.[test]'
            if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed' }
        }
        & $pythonPath tools/benchmark_realtime.py --iterations 150 --output examples/realtime_20261103/benchmark_full_resolution.json
        if ($LASTEXITCODE -ne 0) { throw 'Benchmark failed' }
        & $pythonPath tools/simulate_motion.py --output examples/realtime_20261103 --benchmark examples/realtime_20261103/benchmark_full_resolution.json
        if ($LASTEXITCODE -ne 0) { throw 'Simulation failed' }
    } finally {
        Pop-Location
    }
}
if (-not (Test-Path -LiteralPath $reportFile)) { throw 'Run with -Recompute first or use the supplied replay file.' }
Start-Process -FilePath $reportFile
