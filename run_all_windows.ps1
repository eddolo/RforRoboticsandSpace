<#
=====================================================
Run all R–SO3 Resetability simulations (Windows PowerShell)
Author: Paolo Cappuccini
=====================================================
#>
# If Windows blocks scripts, allow it temporarily:
# Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
# Run it:
# .\run_all_windows.ps1

# --- Setup ----------------------------------------------------
$logDir = "logs"
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }
$logFile = Join-Path $logDir ("run_all_log_" + (Get-Date -Format "yyyyMMdd_HHmmss") + ".txt")

Start-Transcript -Path $logFile -Append | Out-Null
Write-Host "`n🧠  Starting full resetability simulation suite..." -ForegroundColor Cyan

if (-not (Test-Path "results")) { New-Item -ItemType Directory -Path "results" | Out-Null }

# --- Helper for sections -------------------------------------
function Run-Section($emoji, $title, $cmd) {
    Write-Host "`n$emoji  $title" -ForegroundColor Yellow
    try {
        & $cmd | Tee-Object -FilePath $logFile -Append
        Write-Host "✅  Completed $title" -ForegroundColor Green
    } catch {
        Write-Host "❌  Failed: $title" -ForegroundColor Red
        Write-Host $_.Exception.Message -ForegroundColor DarkRed
    }
}

# --- Helper with progress bar for longer tasks ---------------
function Run-WithProgress($emoji, $title, $cmd, $steps = 30) {
    Write-Host "`n$emoji  $title" -ForegroundColor Yellow
    $job = Start-Job -ScriptBlock { & $using:cmd | Tee-Object -FilePath $using:logFile -Append } 
    for ($i = 0; $i -le $steps; $i++) {
        $pct = [math]::Round(($i / $steps) * 100)
        Write-Progress -Activity "$title" -Status "$pct% complete" -PercentComplete $pct
        Start-Sleep -Seconds 1
        if ($job.State -ne 'Running') { break }
    }
    Receive-Job $job -ErrorAction SilentlyContinue
    Remove-Job $job -Force
    Write-Progress -Activity "$title" -Completed
    Write-Host "✅  Completed $title" -ForegroundColor Green
}

# --- Simulation runs -----------------------------------------
Run-Section "🤖" "Robot balance demo (visual)" { python demos\robot_reset_pybullet.py --gui --record }
Run-Section "🛰" "Spacecraft attitude demo" { python demos\spacecraft_reset_demo.py --3d --record }
Run-WithProgress "🚀" "Booster reset Monte Carlo (fast)" { python demos\booster_reset_demo.py --mode fast --vector-pdf --out results\booster_fast }
Run-WithProgress "🚀" "Booster reset Monte Carlo (full)" { python demos\booster_reset_demo.py --mode full --vector-pdf --out results\booster_full }

# --- Locate and open newest results --------------------------
Write-Host "`n📊  Searching for latest outputs..." -ForegroundColor Cyan
$latestVid = Get-ChildItem -Path "videos" -Filter *.mp4 -Recurse -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1
$latestPdf = Get-ChildItem -Path "results\booster_full\plots" -Filter *.pdf -Recurse -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1

if ($latestVid) {
    Write-Host "🎥  Opening video: $($latestVid.Name)" -ForegroundColor Cyan
    Start-Process $latestVid.FullName
}
if ($latestPdf) {
    Write-Host "📄  Opening PDF report: $($latestPdf.Name)" -ForegroundColor Cyan
    Start-Process $latestPdf.FullName
}

Write-Host "`n✅  All simulations completed successfully!" -ForegroundColor Green
Stop-Transcript | Out-Null
Write-Host "🗂️  Log saved to: $logFile" -ForegroundColor DarkGray
Pause
