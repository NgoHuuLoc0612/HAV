# HAV Framework - Windows Launcher
# Starts Python API server and opens the visualizer in browser

param(
    [switch]$NoServer,
    [int]$Port = 7432
)

$Host.UI.RawUI.WindowTitle = "HAV Framework Launcher"

Write-Host ""
Write-Host "  HAV Framework - Launcher" -ForegroundColor Cyan
Write-Host "  ========================" -ForegroundColor DarkCyan
Write-Host ""

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$apiScript  = Join-Path $scriptDir "core\hav_api.py"
$frontendPath = Join-Path $scriptDir "frontend\index.html"

# Check Python
$python = $null
foreach ($cmd in @("python", "python3", "py")) {
    try {
        $ver = & $cmd --version 2>&1
        if ($LASTEXITCODE -eq 0) { $python = $cmd; break }
    } catch {}
}

if (-not $python) {
    Write-Host "  [!] Python not found. Install from https://python.org" -ForegroundColor Red
    Read-Host "  Press Enter to exit"
    exit 1
}

Write-Host "  [+] Python: $($python)" -ForegroundColor Green

# Check for compiled module
$moduleFound = (Get-ChildItem "$scriptDir\core\hav_native*.pyd" -ErrorAction SilentlyContinue) -or
               (Get-ChildItem "$scriptDir\core\hav_native*.so"  -ErrorAction SilentlyContinue)

if ($moduleFound) {
    Write-Host "  [+] Native C++ module found - full performance mode" -ForegroundColor Green
} else {
    Write-Host "  [~] C++ module not built - using pure-Python fallback" -ForegroundColor Yellow
    Write-Host "      Run build.bat to compile for maximum performance" -ForegroundColor DarkYellow
}

# Start API server
if (-not $NoServer) {
    Write-Host "  [+] Starting API server on port $Port..." -ForegroundColor Cyan

    $serverJob = Start-Process -FilePath $python `
        -ArgumentList $apiScript `
        -WorkingDirectory "$scriptDir\core" `
        -PassThru -WindowStyle Minimized

    Start-Sleep -Milliseconds 800

    if ($serverJob.HasExited) {
        Write-Host "  [!] Server failed to start. Check core\hav_api.py" -ForegroundColor Red
    } else {
        Write-Host "  [+] Server running (PID $($serverJob.Id))" -ForegroundColor Green
    }
}

# Open frontend
Write-Host "  [+] Opening visualizer..." -ForegroundColor Cyan

# Enable API mode in the HTML if server is running
if (-not $NoServer) {
    $htmlContent = Get-Content $frontendPath -Raw
    $patched = $htmlContent -replace 'const USE_API\s*=\s*false', 'const USE_API  = true'
    $tempHtml = Join-Path $env:TEMP "hav_index.html"
    $patched | Set-Content $tempHtml -Encoding UTF8
    Start-Process $tempHtml
    Write-Host "  [+] Opened patched HTML (API enabled)" -ForegroundColor Green
} else {
    Start-Process $frontendPath
    Write-Host "  [+] Opened in pure-JS mode" -ForegroundColor Green
}

Write-Host ""
Write-Host "  Visualizer is open in your browser." -ForegroundColor White
Write-Host "  API server: http://127.0.0.1:$Port" -ForegroundColor DarkCyan
Write-Host ""
Write-Host "  Press Ctrl+C or close this window to stop the server." -ForegroundColor DarkGray
Write-Host ""

if (-not $NoServer -and $serverJob -and -not $serverJob.HasExited) {
    try {
        Wait-Process -Id $serverJob.Id -ErrorAction SilentlyContinue
    } catch {
        # User closed window
        if (-not $serverJob.HasExited) {
            Stop-Process -Id $serverJob.Id -Force -ErrorAction SilentlyContinue
        }
    }
}
