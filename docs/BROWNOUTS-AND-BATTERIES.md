# Brownouts and batteries: battery or robot?

Research behind it (WPILib, CTRE, REV, Chief Delphi, 6328 / 1678 / 581 2026 code), 2026-10-28.
Code: `subsystems/PowerMonitor`, `power/BatteryEstimator` (tests), `subsystems/PowerBudget` (guard).
Elastic tab **Battery + Hood**.

## What a brownout is

The roboRIO cuts every motor when the battery dips under its brownout level (6.0 V in our code)
and keeps them off until it is back over 7.5 V. The dip lasts a few hundredths of a second, then
the battery bounces back toward rest. The Driver Station only samples the voltage every 20 ms, so
the number people remember ("it browned out at 12.5 V") is almost always the battery's REST
voltage, not the voltage of the brownout.

How deep the battery dips: volts at the dip = rest volts - current x resistance.
Our robot can pull 280 A+ (drive 4 x 40 A + shooter 2 x 60 A + feeders). With a good battery
(12.7 V, 20 mOhm total) that is 7.1 V; with a tired one (12.5 V, 30 mOhm) the same current hits 4 V.

## Battery or robot: the rule

The robot measures **R = battery + its main wiring** from its own current draw ("Battery R (mOhm)").

- **One battery reads high, the others normal** = that battery (worn, sulfated, or its own SB50/lugs).
- **Every battery reads high** = the robot's main path: 120 A main breaker (the most common
  failure), SB50 contacts worn to copper, a wire stripped too short in a PDH terminal, loose lugs.
- **Normal R but a brownout at 250+ A** = the robot pulled too much at once (pushing, shooter
  spin-up, a stall). The match file names the mechanisms that drew it.
- **Only one motor shows "undervoltage" in its sticky faults** = that motor's branch (breaker, PDH
  terminal, wire), not the battery.

| Check | OK | Watch | Pull from matches |
|---|---|---|---|
| Rest voltage (robot on, disabled, 2 s) | 12.7 V+ | 12.5 to 12.7 | under 12.5 |
| Battery Beak | 15 mOhm or less | 16 to 19 | 20+ |
| Robot "Battery R" (battery + wiring) | under 22 mOhm | 22 to 28 | over 28, or 5+ over the other batteries |

## Pit load test (5 minutes per battery)

1. Battery off the charger 15+ minutes. Read it on the Battery Beak.
2. Put it in the robot, type its name (B01...) in **Match Battery** (Match tab).
3. Robot on blocks, enable, press **Test: Battery Load (shooter 2 s)** three times, 30 s apart.
   Use the middle reading ("Battery Load Test").
4. Test the same known-good battery (call it REF) first every day. If REF goes up 5+ mOhm, the
   robot changed, not the batteries.
5. If several batteries read high: during a load test, a multimeter from the battery posts to the
   PDH input should read under ~1.0 V; each joint (SB50, breaker posts, lugs) well under 0.05 V.
   The joint that reads much more is the fault.

## What every match records (match-data/matches/*.json, "power")

Rest volts before, R, lowest volts (PDH and roboRIO), amps at the lowest moment and the top
mechanisms, the voltage drop to each mechanism, dips under 7 V, brownout snapshots (the worst
moment of the second before each one), sticky undervoltage faults by motor, same battery as last
match, and a verdict. `battery-history.csv` keeps one row per match and load test per battery.

## Prevention in the code

- Brownout guard: under 7.5 V for 0.05 s in any mode = drive supply 25 A per motor until the
  battery is back over 9.5 V for 1 s (Driver left stick click = full power).
- Vacuum/dump modes: drive supply 25 A, 18 A if still sagging.
- Feeders: spindexer and uptake supply 40 A (were 80 and 60).
- Pit Check: battery under 12.5 V, a battery that read high last time, the same battery as the last
  real match, no battery name typed.

## Battery handling at events

Charge at 6 A or less, wait ~15 min after a match before charging (cool case). Charger "green" is
not always full. Use batteries in a fixed rotation, log every match (the robot does it if Match
Battery is typed). Cold batteries lose capacity: keep them inside, off the floor.
