# TECHMEN 5669: copy every match the robot recorded into this repo's match-data folder, then
# (when the laptop is online) commit and push the small files so the team and Claude can read them.
# Run it with the robot on (Wi-Fi or USB). Safe to run any number of times: it skips what it has.
$ErrorActionPreference = "Stop"
$repo = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$out = Join-Path $repo "match-data"
$robot = $null
# Any answer from the robot's web server counts (even "not found": the match list only exists
# after the first saved match).
function Test-Robot($ip) {
    try { Invoke-WebRequest "http://$($ip):5805/" -TimeoutSec 2 -UseBasicParsing | Out-Null; return $true }
    catch { return [bool]$_.Exception.Response }
}
foreach ($ip in @("10.56.69.2", "172.22.11.2")) { if (Test-Robot $ip) { $robot = $ip; break } }
if (-not $robot) { Write-Host "Robot not reachable (10.56.69.2 or 172.22.11.2). Turn it on and connect." -ForegroundColor Yellow; exit 1 }
Write-Host "Robot at $robot"
New-Item -ItemType Directory -Force (Join-Path $out "matches"), (Join-Path $out "logs") | Out-Null

$names = @()
try { $names = @((Invoke-WebRequest "http://$($robot):5805/matches/index.json" -UseBasicParsing).Content | ConvertFrom-Json) }
catch { Write-Host "No matches saved on the robot yet." }
$new = 0
foreach ($n in $names) {
    $dest = Join-Path $out "matches\$n"
    if (-not (Test-Path $dest)) { Invoke-WebRequest "http://$($robot):5805/matches/$n" -OutFile $dest -UseBasicParsing; $new++ }
}
Write-Host "Match files: $($names.Count) on the robot, $new new"
foreach ($f in @("matches.csv", "shot-log.csv", "robot-settings.json")) {
    try { Invoke-WebRequest "http://$($robot):5805/$f" -OutFile (Join-Path $out $f) -UseBasicParsing } catch {}
}
# Full logs (.wpilog) for AdvantageScope: port 5807, only ones not copied yet.
try {
    $logs = (Invoke-WebRequest "http://$($robot):5805/logs-index.json" -UseBasicParsing).Content | ConvertFrom-Json
    $got = 0
    foreach ($l in $logs) {
        $dest = Join-Path $out "logs\$($l.name)"
        if (-not (Test-Path $dest) -or (Get-Item $dest).Length -ne $l.bytes) {
            try { Invoke-WebRequest "http://$($robot):5807/$($l.name)" -OutFile $dest -UseBasicParsing; $got++ }
            catch { Write-Host "  skipped $($l.name) (renamed or gone on the robot)" }
        }
    }
    Write-Host "Logs: $got copied to match-data\logs"
} catch { Write-Host "Logs: none listed yet (a USB stick in the roboRIO keeps them)" }

# Push the small files (not the logs) to GitHub so the team and Claude can read them.
# Bug hunt Oct 28: this used to commit and "git pull" in THIS folder, the one the robot code is
# deployed from, so a sync could pull in code nobody tested at the event. Now it never touches
# this folder's git at all: it uses its own copy of the repo in %LOCALAPPDATA%\Techmen5669-MatchData
# (made the first time), which only ever holds what is on GitHub plus these match files.
# Windows PowerShell turns git's normal stderr chatter into errors under "Stop", so git runs under
# "Continue" and is judged by its exit code.
$ErrorActionPreference = "Continue"
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Write-Host "Git isn't installed on this laptop: the files are in $out (send that folder another way)." -ForegroundColor Yellow
    Write-Host "Done. Folder: $out"; exit 0
}
$url = "https://github.com/FRC-Team-5669-Techmen/2026-5669-Official.git"
$data = Join-Path $env:LOCALAPPDATA "Techmen5669-MatchData"
function Git-Data { & git -C $data @args 2>&1 | Out-Null; return ($LASTEXITCODE -eq 0) }
$ok = $true
if (-not (Test-Path (Join-Path $data ".git"))) {
    Write-Host "First sync on this laptop: making the match-data copy of the repo (needs internet)..."
    & git clone -q $url $data 2>&1 | Out-Null
    $ok = ($LASTEXITCODE -eq 0)
}
if ($ok) {
    Git-Data config user.name "Techmen 5669 match data" | Out-Null
    Git-Data config user.email "frc5669-match-data@users.noreply.github.com" | Out-Null
}
$pushed = $false
for ($try = 1; $ok -and $try -le 2 -and -not $pushed; $try++) {
    # The copy only mirrors GitHub; every match file is copied in again from $out each time.
    if (-not (Git-Data fetch -q origin main)) { $ok = $false; break }
    if (-not (Git-Data reset -q --hard origin/main)) { $ok = $false; break }
    $dm = Join-Path $data "match-data"
    New-Item -ItemType Directory -Force (Join-Path $dm "matches") | Out-Null
    Copy-Item (Join-Path $out "matches\*") (Join-Path $dm "matches") -Force -ErrorAction SilentlyContinue
    foreach ($f in @("matches.csv", "shot-log.csv", "robot-settings.json")) {
        if (Test-Path (Join-Path $out $f)) { Copy-Item (Join-Path $out $f) $dm -Force }
    }
    Git-Data add -A -- match-data | Out-Null
    & git -C $data diff --cached --quiet
    if ($LASTEXITCODE -eq 0) { Write-Host "GitHub already has all of it."; $pushed = $true; break }
    if (-not (Git-Data commit -q -m "Match data from the robot ($(Get-Date -Format 'yyyy-MM-dd HH:mm'))")) { $ok = $false; break }
    if (Git-Data push -q origin HEAD:main) { $pushed = $true; Write-Host "Pushed to GitHub: Claude and the team can read it." -ForegroundColor Green }
}
if (-not $pushed) {
    Write-Host "Not pushed (no internet or GitHub sign-in): the files are safe in $out. Run this again online." -ForegroundColor Yellow
}
Write-Host "Done. Folder: $out"
