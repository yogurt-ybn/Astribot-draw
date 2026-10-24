$ErrorActionPreference = 'Stop'
$moduleRoot = $PSScriptRoot
$projectRoot = Split-Path -Parent $moduleRoot
$pythonOptions = @()
$taskPython = $null

if ($env:LIGHTPAINT_PYTHON) {
    $taskPython = (Get-Command $env:LIGHTPAINT_PYTHON -ErrorAction Stop).Source
} else {
    foreach ($candidate in @(
        (Join-Path $moduleRoot '.venv/Scripts/python.exe'),
        (Join-Path $projectRoot '.venv/Scripts/python.exe')
    )) {
        if (Test-Path -LiteralPath $candidate) {
            $taskPython = $candidate
            break
        }
    }
    if (-not $taskPython) {
        $launcher = Get-Command py -ErrorAction SilentlyContinue
        if ($launcher) {
            $taskPython = $launcher.Source
            $pythonOptions = @('-3')
        } else {
            $taskPython = (Get-Command python -ErrorAction Stop).Source
        }
    }
}

& $taskPython @pythonOptions -c 'import cv2, numpy, PIL, flask, waitress'
if ($LASTEXITCODE -ne 0) {
    throw 'Missing dependencies. Run: python -m pip install -r camera_lightpainting/requirements.txt'
}

Push-Location -LiteralPath $moduleRoot
try {
    & $taskPython @pythonOptions (Join-Path $moduleRoot 'web_app.py') @args
    if ($LASTEXITCODE -ne 0) {
        throw 'Light painting exited with an error. See the Python traceback above.'
    }
} finally {
    Pop-Location
}
