# TECHMEN 5669: copy every match the robot recorded into this repo's match-data folder, then
# (when the laptop is online) commit and push the small files so the team and Claude can read them.
# Run it with the robot on (Wi-Fi or USB). Safe to run any number of times: it skips what it has.
$ErrorActionPreference = "Stop"
$repo = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$out = Join-Path $repo "match-data"
$robot = $null
foreach ($ip in @("10.56.69.2", "172.22.11.2")) {
    try { Invoke-WebRequest "http://$($ip):5805/matches/index.json" -TimeoutSec 2 -UseBasicParsing | Out-Null; $robot = $ip; break } catch {}
}
if (-not $robot) { Write-Host "Robot not reachable (10.56.69.2 or 172.22.11.2). Turn it on and connect." -ForegroundColor Yellow; exit 1 }
Write-Host "Robot at $robot"
New-Item -ItemType Directory -Force (Join-Path $out "matches"), (Join-Path $out "logs") | Out-Null

$names = (Invoke-WebRequest "http://$($robot):5805/matches/index.json" -UseBasicParsing).Content | ConvertFrom-Json
$new = 0
foreach ($n in $names) {
    $dest = Join-Path $out "matches\$n"
    if (-not (Test-Path $dest)) { Invoke-WebRequest "http://$($robot):5805/matches/$n" -OutFile $dest -UseBasicParsing; $new++ }
}
Write-Host "Match files: $($names.Count) on the robot, $new new"
foreach ($f in @("matches.csv", "shot-log.csv")) {
    try { Invoke-WebRequest "http://$($robot):5805/$f" -OutFile (Join-Path $out $f) -UseBasicParsing } catch {}
}
# Full logs (.wpilog) for AdvantageScope: port 5807, only ones not copied yet.
try {
    $logs = (Invoke-WebRequest "http://$($robot):5805/logs-index.json" -UseBasicParsing).Content | ConvertFrom-Json
    $got = 0
    foreach ($l in $logs) {
        $dest = Join-Path $out "logs\$($l.name)"
        if (-not (Test-Path $dest) -or (Get-Item $dest).Length -ne $l.bytes) {
            Invoke-WebRequest "http://$($robot):5807/$($l.name)" -OutFile $dest -UseBasicParsing; $got++
        }
    }
    Write-Host "Logs: $got copied to match-data\logs"
} catch { Write-Host "Logs: none listed yet (a USB stick in the roboRIO keeps them)" }

# Commit and push the small files (not the logs) when git and the internet are there.
Set-Location $repo
$isRepo = $false
try { $isRepo = ((git rev-parse --is-inside-work-tree 2>$null) -eq "true") } catch {}
if ($isRepo) {
    git add match-data/matches match-data/matches.csv match-data/shot-log.csv 2>$null
    $changes = git diff --cached --name-only
    if ($changes) {
        git commit -q -m "Match data from the robot ($(Get-Date -Format 'yyyy-MM-dd HH:mm'))"
        try { git pull -q --rebase; git push -q; Write-Host "Pushed to GitHub: Claude and the team can read it." -ForegroundColor Green }
        catch { Write-Host "Saved and committed here; push when the laptop is online (run this again)." -ForegroundColor Yellow }
    } else { Write-Host "Nothing new to commit." }
}
if (-not $isRepo) { Write-Host "This folder is not a git clone: send the match-data folder another way." -ForegroundColor Yellow }
Write-Host "Done. Folder: $out"
