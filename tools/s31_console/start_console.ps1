<#
.SYNOPSIS
  Start the S31 Console (flash + serial monitor web UI) on http://127.0.0.1:8765
.EXAMPLE
  .\start_console.ps1            # real board on COM5 @ 921600
  .\start_console.ps1 -Mock      # simulator, no hardware touched
  .\start_console.ps1 -Port COM7 -Baud 115200 -LogOnStart
  .\start_console.ps1 -Project E:\ESP32-S31-CSBAura\s31-btstack-speaker   # flash another project
  .\start_console.ps1 -Project ... -IdfPath D:\ESP32-S31\esp-idf-v6.1-latest   # build with another ESP-IDF tree
#>
param(
  [switch]$Mock,
  [string]$Port = "COM5",
  [int]$Baud = 921600,
  [int]$HttpPort = 8765,
  [switch]$LogOnStart,
  [switch]$NoAutoLog,
  [switch]$NoBrowser,
  [string]$Project = "",
  [string]$IdfPath = ""
)
# newer ESP-IDF refuses to export when MSYSTEM is set (e.g. started from Git Bash)
Remove-Item Env:MSYSTEM -ErrorAction SilentlyContinue
$py = Join-Path $env:USERPROFILE ".espressif\python_env\idf6.1_py3.12_env\Scripts\python.exe"
if (-not (Test-Path $py)) { Write-Warning "IDF python env not found at $py - falling back to 'python'"; $py = "python" }
$server = Join-Path $PSScriptRoot "server.py"
$argList = @($server, "--port", $Port, "--baud", $Baud, "--http-port", $HttpPort)
if ($Mock)       { $argList += "--mock" }
if ($LogOnStart) { $argList += "--log-on-start" }
if ($NoAutoLog)  { $argList += "--no-auto-log" }
if ($Project)    { $argList += @("--project", $Project) }
if ($IdfPath)    { $argList += @("--idf-path", $IdfPath) }
if (-not $NoBrowser) {
  # open the UI shortly after the server comes up
  Start-Job -ScriptBlock { param($u) Start-Sleep -Seconds 1.5; Start-Process $u } -ArgumentList "http://127.0.0.1:$HttpPort/" | Out-Null
}
& $py @argList
