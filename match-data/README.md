# Match data

Filled by `tools/match-data/Sync Match Data.bat` on the driver station laptop.

- `matches/*.json`: one file per match (practice, real, auto tests, teleop sessions over 20 s):
  scoring by phase, battery (start, min and where, brownouts), drive current, ready-to-fire time and
  why it wasn't ready, aim and vision, climb times, CAN health, and a timeline of every mode change
  and interrupted command. Schema: `src/main/java/frc/robot/practice/MatchTelemetry.java`.
- `matches.csv`: one row per match. `shot-log.csv`: every shot.
- `robot-settings.json`: every value saved on the roboRIO (recorded shot, pass and climb tables,
  feed volts, turret far end, backup spots, switches) with the code version. If the roboRIO is ever
  wiped, the numbers are here.
- `logs/*.wpilog` (not committed, too big): every signal, for AdvantageScope.
