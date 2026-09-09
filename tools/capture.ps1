<#
  capture.ps1 - pull the flight recorder off the board over SWD.

  Drive the car (or hold it over the track in bench mode), plug the USB cable back
  into the MCU-Link, and run this. It reads the ring buffer out of SRAMX and decodes
  it. Nothing is flashed and the chip is not reset, so the log survives - and even if
  something does reset it, the buffer is in a no-init section and stays put.

    .\tools\capture.ps1                 capture and print the audit
    .\tools\capture.ps1 -Csv run1.csv   also write every frame to CSV
    .\tools\capture.ps1 -KeepBin        keep the raw dump
#>
param(
    [string]$Csv = "",
    [switch]$KeepBin,
    [int]$Port = 3333,
    [string]$LinkServer = "C:\NXP\LinkServer_26.6.137\LinkServer.exe",
    [string]$Gdb = "$env:USERPROFILE\.mcuxpressotools\arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin\arm-none-eabi-gdb.exe"
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$proj = Split-Path -Parent $here
$elf  = Join-Path $proj "Debug\nxpcup_official.axf"
$bin  = Join-Path $here  "telemetry.bin"
$gdbScript = Join-Path $here "_dump.gdb"

foreach ($p in @($LinkServer, $Gdb, $elf)) {
    if (-not (Test-Path $p)) { Write-Error "Not found: $p"; exit 1 }
}

# The buffer is one contiguous struct, so its own symbol bounds the dump exactly.
@"
set confirm off
set pagination off
target remote localhost:$Port
dump binary memory "$($bin -replace '\\','/')" &g_tlm ((char *)&g_tlm + sizeof(g_tlm))
detach
quit
"@ | Set-Content -Path $gdbScript -Encoding ascii

# Only one thing may own the probe at a time, and a debug session left running in the
# IDE is the usual reason this script cannot see it.
$holders = Get-Process -ErrorAction SilentlyContinue |
           Where-Object { $_.ProcessName -match "redlinkserv|crt_emu_cm_redlink" }
if ($holders) {
    Write-Host "The debug probe is already in use by a running debug session." -ForegroundColor Yellow
    Write-Host "Stop the debugger in VS Code (the red square), then run this again." -ForegroundColor Yellow
    Write-Host ("Holding processes: " + (($holders | ForEach-Object { "$($_.ProcessName)($($_.Id))" }) -join ", "))
    exit 1
}

Write-Host "Starting gdbserver..." -ForegroundColor Cyan
$srv = Start-Process -FilePath $LinkServer `
    -ArgumentList @("gdbserver", "--gdb-port", "$Port", "MCXN947:FRDM-MCXN947") `
    -PassThru -WindowStyle Hidden

try {
    # Give the server a moment to bind the port before gdb connects.
    $deadline = (Get-Date).AddSeconds(20)
    $up = $false
    while ((Get-Date) -lt $deadline) {
        try {
            $c = New-Object Net.Sockets.TcpClient
            $c.Connect("127.0.0.1", $Port)
            $c.Close()
            $up = $true
            break
        } catch { Start-Sleep -Milliseconds 300 }
    }
    if (-not $up) { Write-Error "gdbserver did not open port $Port"; exit 1 }

    Write-Host "Dumping SRAMX..." -ForegroundColor Cyan
    & $Gdb -q -batch -x $gdbScript $elf 2>&1 | Where-Object { $_ -match "error|Error|warning" } | ForEach-Object { Write-Host $_ }
}
finally {
    if ($srv -and -not $srv.HasExited) { Stop-Process -Id $srv.Id -Force -ErrorAction SilentlyContinue }
    Remove-Item $gdbScript -ErrorAction SilentlyContinue
}

if (-not (Test-Path $bin)) { Write-Error "No dump produced."; exit 1 }
Write-Host ("Got {0:N0} bytes" -f (Get-Item $bin).Length) -ForegroundColor Green
Write-Host ""

$py = Join-Path $here "decode_telemetry.py"
$args = @($py, $bin)
if ($Csv -ne "") { $args += @("--csv", $Csv) }
& python @args

if (-not $KeepBin) { Remove-Item $bin -ErrorAction SilentlyContinue }
