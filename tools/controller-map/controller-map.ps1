# Techmen 5669 Controller Map launcher
# Reads both Xbox controllers through Windows XInput (works no matter which window has
# focus, so Driver Station can stay selected), serves the map page on this laptop only
# (127.0.0.1:5669, nothing leaves the machine), and opens it in a Chrome or Edge app
# window placed right under the docked Driver Station.

$ErrorActionPreference = 'Stop'
$dir  = Split-Path -Parent $MyInvocation.MyCommand.Path
$port = 5669
$url  = "http://127.0.0.1:$port/"
try { Start-Transcript -Path (Join-Path $dir 'controller-map.log') -Force | Out-Null } catch { }
trap { Write-Host ""; Write-Host "  ERROR: $($_.Exception.Message)" -ForegroundColor Red; Write-Host "  at line $($_.InvocationInfo.ScriptLineNumber)"; try { Stop-Transcript | Out-Null } catch { }; Read-Host "  Press Enter to close"; exit 1 }

Add-Type -TypeDefinition @"
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class Techmen {
  [StructLayout(LayoutKind.Sequential)] public struct GP { public ushort b; public byte lt; public byte rt; public short lx; public short ly; public short rx; public short ry; }
  [StructLayout(LayoutKind.Sequential)] public struct ST { public uint pkt; public GP g; }
  [DllImport("xinput1_4.dll", EntryPoint="XInputGetState")] static extern int Get14(int i, out ST s);
  [DllImport("xinput9_1_0.dll", EntryPoint="XInputGetState")] static extern int Get91(int i, out ST s);
  static int mode = 0;
  static int Get(int i, out ST s) {
    if (mode == 0) { try { return Get14(i, out s); } catch { mode = 1; } }
    try { return Get91(i, out s); } catch { s = new ST(); return 1; }
  }
  public static string Json() {
    var sb = new StringBuilder("[");
    for (int i = 0; i < 4; i++) {
      ST s; int r = Get(i, out s);
      if (i > 0) sb.Append(',');
      if (r != 0) { sb.Append("null"); continue; }
      sb.AppendFormat("{{\"i\":{0},\"b\":{1},\"lt\":{2},\"rt\":{3},\"lx\":{4},\"ly\":{5},\"rx\":{6},\"ry\":{7}}}", i, s.g.b, s.g.lt, s.g.rt, s.g.lx, s.g.ly, s.g.rx, s.g.ry);
    }
    return sb.Append("]").ToString();
  }

  // Legacy joystick API (same family Driver Station uses). Works in the background.
  [StructLayout(LayoutKind.Sequential)] public struct JOY { public int size, flags, x, y, z, r, u, v, buttons, bnum, pov, res1, res2; }
  [DllImport("winmm.dll")] static extern int joyGetPosEx(int id, ref JOY j);
  public static string JsonMM() {
    var sb = new StringBuilder("[");
    bool first = true;
    for (int i = 0; i < 16; i++) {
      JOY j = new JOY(); j.size = Marshal.SizeOf(typeof(JOY)); j.flags = 0xFF;
      int r; try { r = joyGetPosEx(i, ref j); } catch { r = 1; }
      if (r != 0) continue;
      if (!first) sb.Append(','); first = false;
      sb.AppendFormat("{{\"i\":{0},\"b\":{1},\"pov\":{2},\"x\":{3},\"y\":{4},\"z\":{5},\"r\":{6},\"u\":{7}}}", i, j.buttons, j.pov, j.x, j.y, j.z, j.r, j.u);
    }
    return sb.Append("]").ToString();
  }
  public static string State() { return "{\"v\":2,\"xi\":" + Json() + ",\"mm\":" + JsonMM() + "}"; }

  // window placement
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
  delegate bool EnumProc(IntPtr h, IntPtr p);
  [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc f, IntPtr p);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr h, int x, int y, int w, int hh, bool repaint);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
  [DllImport("user32.dll")] static extern bool SystemParametersInfo(int a, int b, out RECT r, int c);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  public static IntPtr Find(string prefix) {
    IntPtr found = IntPtr.Zero;
    EnumWindows((h, p) => {
      if (!IsWindowVisible(h)) return true;
      var sb = new StringBuilder(256); GetWindowText(h, sb, 256);
      if (sb.ToString().StartsWith(prefix)) { found = h; return false; }
      return true;
    }, IntPtr.Zero);
    return found;
  }
  public static RECT WorkArea() { RECT r; SystemParametersInfo(0x30, 0, out r, 0); return r; }
}
"@

try { [void][Techmen]::SetProcessDPIAware() } catch { }

function Open-MapWindow {
  try {
  $browsers = @(
    "$env:ProgramFiles\Google\Chrome\Application\chrome.exe",
    "${env:ProgramFiles(x86)}\Google\Chrome\Application\chrome.exe",
    "$env:LOCALAPPDATA\Google\Chrome\Application\chrome.exe",
    "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe",
    "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe"
  )
  $exe = $browsers | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
  if (-not $exe) { Start-Process $url; return }
  $profileDir = Join-Path $env:LOCALAPPDATA 'Techmen5669ControllerMap'
  $argList = @("--app=$url", "--user-data-dir=`"$profileDir`"", '--no-first-run', '--no-default-browser-check',
            '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
            '--disable-backgrounding-occluded-windows')
  Start-Process -FilePath $exe -ArgumentList $argList
  $script:placeBy = (Get-Date).AddSeconds(20)
  } catch { Write-Host "Could not open or place the window automatically. Open $url in Chrome." }
}

# Move the map window under the docked Driver Station (or fill the work area).
$script:placeBy = $null
function Try-PlaceWindow {
  if (-not $script:placeBy) { return }
  if ((Get-Date) -gt $script:placeBy) { $script:placeBy = $null; Write-Host "  Could not find the map window to place it. Drag it under Driver Station."; return }
  try {
    $h = [Techmen]::Find('Techmen Controller Map')
    if ($h -eq [IntPtr]::Zero) { return }
    $wa = [Techmen]::WorkArea()
    $top = $wa.T
    $ds = [Techmen]::Find('FRC Driver Station')
    if ($ds -ne [IntPtr]::Zero) {
      $r = New-Object Techmen+RECT
      [void][Techmen]::GetWindowRect($ds, [ref]$r)
      $docked = ($r.T -le $wa.T + 10) -and (($r.R - $r.L) -ge (($wa.R - $wa.L) * 0.8))
      if ($docked) { $top = $r.B }
    }
    [void][Techmen]::ShowWindow($h, 9)
    [void][Techmen]::MoveWindow($h, $wa.L, $top, $wa.R - $wa.L, $wa.B - $top, $true)
    Write-Host "  Map window placed under Driver Station."
  } catch { Write-Host "  Window placement failed: $($_.Exception.Message)" }
  $script:placeBy = $null
}

# Find a port: reuse a map that is already running, else take the first free one.
$listener = $null
foreach ($p in 5669..5679) {
  try {
    $r = Invoke-WebRequest -Uri "http://127.0.0.1:$p/state" -UseBasicParsing -TimeoutSec 2
    if ($r.Content -like '*"v":2*') {
      $url = "http://127.0.0.1:$p/"
      Write-Host "Controller Map is already running on port $p. Opening the window."
      Open-MapWindow
      while ($script:placeBy) { Start-Sleep -Milliseconds 300; Try-PlaceWindow }
      exit
    }
  } catch { }
  try {
    $l = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Loopback, $p)
    $l.Start()
    $listener = $l; $port = $p; $url = "http://127.0.0.1:$p/"
    break
  } catch { Write-Host "Port $p is busy: $($_.Exception.Message)" }
}
if (-not $listener) { throw "No free port between 5669 and 5679." }

$htmlPath = Join-Path $dir 'index.html'
if (-not (Test-Path $htmlPath)) { throw "index.html is missing from $dir" }
Write-Host ""
Write-Host "  TECHMEN 5669 CONTROLLER MAP" -ForegroundColor Yellow
Write-Host "  Reading Xbox controllers through XInput. Serving $url"
Write-Host "  XInput controllers: $([Techmen]::Json())"
Write-Host "  Joystick controllers: $([Techmen]::JsonMM())"
Write-Host "  Leave this window open (minimize it). Close it to stop the map."
Write-Host ""
Open-MapWindow

$ascii = [System.Text.Encoding]::ASCII
$served = 0
while ($true) {
  if (-not $listener.Pending()) { Try-PlaceWindow; Start-Sleep -Milliseconds 5; continue }
  $client = $listener.AcceptTcpClient()
  try {
    $client.ReceiveTimeout = 2000
    $ns = $client.GetStream()
    $reader = New-Object System.IO.StreamReader($ns, $ascii, $false, 2048, $true)
    $line = $reader.ReadLine()
    while ($true) { $hl = $reader.ReadLine(); if ($hl -eq $null -or $hl -eq '') { break } }
    $path = '/'
    if ($line) { $parts = $line.Split(' '); if ($parts.Length -ge 2) { $path = $parts[1] } }
    $status = '200 OK'
    if ($path.StartsWith('/state')) {
      $body = [System.Text.Encoding]::UTF8.GetBytes([Techmen]::State()); $type = 'application/json'
    } elseif ($path -eq '/' -or $path.StartsWith('/index.html') -or $path.StartsWith('/?')) {
      $body = [System.IO.File]::ReadAllBytes($htmlPath); $type = 'text/html; charset=utf-8'
      Write-Host "  Map page loaded."
    } else {
      $status = '404 Not Found'; $body = [byte[]]@(); $type = 'text/plain'
    }
    $head = "HTTP/1.1 $status`r`nContent-Type: $type`r`nContent-Length: $($body.Length)`r`nCache-Control: no-store`r`nConnection: close`r`n`r`n"
    $hb = $ascii.GetBytes($head)
    $ns.Write($hb, 0, $hb.Length)
    if ($body.Length -gt 0) { $ns.Write($body, 0, $body.Length) }
    $ns.Flush()
    $served++
    if ($served -eq 50) { Write-Host "  Controllers streaming to the map." }
  } catch { Write-Host "  Request error: $($_.Exception.Message)" }
  finally { $client.Close() }
}
