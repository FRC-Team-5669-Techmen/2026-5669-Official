# CLAUDE.md - FRC Team 5669 Techmen (2026 Season)

## Build & Deploy

```bash
# Build the project
./gradlew build

# Deploy to the RoboRIO
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Run tests
./gradlew test
```

- **GradleRIO version**: 2026.2.1
- **Java**: 17
- **Team number**: loaded from `.wpilib/wpilib_preferences.json`
- Main class: `frc.robot.Main`

## Pit Check (read it before every match)

`subsystems/PitCheck` checks the robot once a second, always (no button), and only reads: dashboard **"Pit Check"** = `ALL GO` or the list of problems; **"Pit Check Details"** shows every item; changes print to the DS console as `PIT CHECK |`. Checks: battery >= 12.4 V while disabled, all 23 CAN devices answering (IDs 1-22 + Pigeon 41), Limelight heartbeat, Practice Field switch OFF when an FMS is attached, alliance set, turret far end set, pose trusted, shot table has 4+ recorded points, roboRIO disk >= 50 MB free.

## Shot Map Day (Oct 26) and backup shots

**Guide for the students: `tools/shot-map/Shot Map Day Guide.html`** (open it with `tools/shot-map/Start Shot Map Day.bat`, which also opens the Driver Station, Elastic and the controller map). Order: deploy newest code, regear the spindexer 64:1 to 16:1 (remove one 4:1 MAXPlanetary stage, shorter screws), power on with zeros, Elastic Ctrl+D loads `src/main/deploy/elastic-layout.json` (tabs "Shot Map" and "Feed + Spots"; served by `WebServer` on port 5800 in `Robot`), Pit Check, tune feed volts, save backup spots, record the shot map, test, send data.

- `subsystems/ShotMapGuide`: "Shot Map Step" (one plain-English instruction), "Shot Map Progress" (checklist of distances: the 3 backup-spot distances + 1.5/2.25/3.0/3.75/4.5/5.25 m not near them), pre-fills Tune RPM/Hood from the table at each new distance, writes **every shot and every Record press to `/home/lvuser/shotmap/shot-log.csv`**, downloadable at **http://10.56.69.2:5805/shot-log.csv** (WebServer port 5805). Columns are in the CSV header.
- "Record Shot Point" now also prints `SHOT TABLE | ...` and `SHOT TABLE CODE | t.put(new Point(...)); ...` to the console; dashboard "Shot Table Code" has the same Java. **To make a session permanent: paste that into `ShotTable.defaults()`** (recorded points also live in roboRIO Preferences "Shot Table Recorded", used immediately).
- Feed volts "Spindex Forward Volts" / "Uptake Forward Volts" are saved in roboRIO Preferences when changed on the dashboard (survive reboots); copy the tuned values into `Constants.Index.kForwardSpeed` / `Constants.ShooterIntake.kForwardVolts`. Spindex stator limit 80 A (was 160) for the 16:1 gearing.
- **Backup shots** (`subsystems/BackupShots`, camera-proof): park on the spot, press **Operator A = Midshot** (between our hub and our climb), **Operator X = Left trench**, **Operator LB = Right trench** (left/right as seen from our driver station). It resets the pose to that spot (keeps the gyro heading; if the heading is unknown it assumes the robot faces straight away from our DS), turns auto-aim on (shared `aimCommand` with Operator RB), then RT fires as usual. Spots are stored relative to our hub (forward toward our DS wall, left = driver's left) so they mirror for red/blue; defaults are estimates (Midshot 2.30 m forward; trenches 1.05 m forward, 3.39 m left/right); **"Save Spot: ..." dashboard buttons store the real spot** (Preferences "Backup Spot Midshot" etc., needs Pose Trusted).
- Driver Station keys: **Enter = Disable, Space = Emergency Stop** (needs a roboRIO reboot after).

## Automatic agitation (Oct 26)

No buttons: whenever a shooting command feeds (Operator RT with or without auto-aim, autos `score`/`shoot`), the robot keeps the hopper flowing by itself. Code: `frc.robot.feeding` (pure logic, tests in `src/test/java/frc/robot/feeding/FeedingTest.java`), `subsystems/Agitator` (runs every loop, no motors of its own), `IndexSubsystem.feed()`, `GroundIntakePivotSubsystem.pumpCommand()`. Numbers in `Constants.Agitation`. Research behind it (Chief Delphi REBUILT threads, Team 581's 2026 code, granular-flow studies, vendor ratings): `docs/AGITATION-RESEARCH.md`.

- **Two failure modes**: **bridging** = balls wedge above the spindexer, it spins empty; **inlet jam** = two balls fight for the uptake and the spindexer stalls.
- **Prevent** (Team 581's pattern): while balls are fed and the intake is down, the roller turns in at **"Roller Assist Volts"** (5 V, saved; 0 = off).
- **Gap ladder** (`AgitationPlanner`), reset by every ball (`ShotDetector`: flywheel falls 120 RPM below its recent peak, works even below target). The gap clock runs only while actually feeding (ShootCommand READY), so waiting for aim never triggers it:
  1. **0.35 s** no ball: **nudge** = roller taps out (-4 V, 0.08 s; -9 V/0.15 s threw a ball out of the robot Oct 26) then in, spindexer keeps feeding. Intake stowed: 0.15 s spindexer reverse shake.
  2. **0.8 s**: **pump** the intake ("Agitate Lift (rot)" 2.0 of ~5.7, up and down twice, Motion Magic both ways, ends resting DOWN) with the roller blink. Intake can't pump: spindexer shake.
  3. Pump brought no ball: **shimmy** the drivetrain, only with **"Agitate Drive Shimmy"** ON (saved, **OFF by default**, teleop only, driver's sticks centered; any stick cancels it): robot-relative forward/back 0.3 m/s at 25 rad/s (~1.2 cm each way) for 0.4 s, shoot-while-moving lead off during it. Never a rotation wiggle: that spins the turret past its 30 deg/s READY limit and feeding stops.
  4. 3 big agitations with no ball: hopper probably **EMPTY**, stops.
  Also: 3 spindexer jams within 3 s trigger a pump.
- **Inlet jam -> unjam**: `IndexSubsystem.feed()` (all shooting feeds; Operator B and rewinds still use `run()`): over 45 A while under 8 motor rot/s for 0.15 s (ignored the first 0.3 s) = reverse -6 V for 0.25 s. 3 jams within 3 s = no auto-reverse for 3 s (the 80 A stator limit holds it). 0.08 s voltage ramp softens the flips. Dashboard "Spindex Unjams".
- **Protection**: no pumps while a pivot or spindexer motor is over 80 C (until under 65 C), more than 12 pumps per minute, or battery under 8.5 V; no shimmy on low battery. Pivot stays at 40 A stator (~14 Nm per Thrifty cycloidal, rated ~75 Nm usable; don't exceed 60-70 A). Dashboard "Agitator Max Motor Temp (C)". Note: CTRE swerve applies `kSlipCurrent` (120 A) as the drive stator limit, not the 80 A in `TunerConstants`.
- **Driver always wins**: Driver A or the intake jog interrupts a pump; the roller is left alone while the driver holds a roller trigger; sticks cancel a shimmy.
- Dashboard (tab "Feed + Spots"): "Auto Agitate", "Agitate Drive Shimmy", "Roller Assist Volts", "Agitate Lift (rot)" (all saved), "Agitator" (what it's doing), **"Last Volley"** (balls, seconds, balls/s, longest gap, nudges, pumps, shakes, shimmies, unjams: the scorecard). DogLog `Feed/*`.
- **Tuning on the robot**: same full hopper, compare "Last Volley" with Auto Agitate OFF, ON, and ON + shimmy. Longest gap still over ~0.5 s = raise the lift a little; intake slamming = lower it. Unjams during normal feeding = raise `kJamAmps`. Balls counted far below balls shot = lower `kShotDropRpm`. If gaps keep coming from the same spot, the fix is mechanical (star wheel or narrower inlet where they bridge).

## Passing, vacuum mode, shoot on the move, climb + trench guards (Oct 27)

Research behind all of it (6328, 1678, 581, 5000 public 2026 code, the TU22 game manual): `docs/PASSING-AND-SOTM-RESEARCH.md`. Code: `aiming/TargetPicker` (pure, tests `TargetPickerTest`), `aiming/ShotSolver` (pure, tests `ShotSolverTest`), `subsystems/AimSubsystem`, `commands/VacuumCommand`, `HubShiftTracker.computeHubStateAt`. Numbers in `Constants.Aim`.

- **What to shoot at** (`TargetPicker`, every loop): **SCORE** into our hub while the robot CENTER is within **4.19 m of our wall**, checked now and at the predicted launch spot 0.3 s ahead. Reason: **G407**, launching into our hub is legal only with the bumpers partly inside our alliance zone (4.029 m), MAJOR FOUL otherwise; 4.34 m guarantees it at any heading for our 29.5 x 24.5 in bumpers, minus 0.15 m for pose error. Beyond the line: **PASS** to a landing point in a corner of our alliance zone on the robot's side: 0.94 m from our wall, 1.65 m from the side wall (6328's point). If the straight line crosses either hub (+0.30 m), it tries the backup (1.40 m / 1.00 m, 581's), then the other side's corner and backup; all blocked = hold ("PASS: a hub blocks every lane: move sideways"), since a lob into our hub from outside the zone is also a G407 foul. Mirrored for red. 0.25 m hysteresis; the pass point is frozen for the length of a volley, but its lane is re-checked every loop.
- **Operator Start** cycles the target mode **AUTO -> SCORE only -> PASS only** (dashboard "Shot Target" shows target and mode; chooser "Shot Target Mode" too). Forced SCORE still refuses to shoot outside the zone ("SCORE: outside our alliance zone (G407 foul)").
- **Operator Back = VACUUM MODE** (toggle): intake down (not while climbing), roller in, auto-aim on, shooter at the table speed, and it **fires by itself whenever ready**: score inside our zone, pass home from the neutral zone. The drive team always wins: Driver RT/LT take the roller, Operator RT/B take the feeders, the jogs take aim, only while held; vacuum picks each piece back up when it's free (the shooter keeps spinning through an unjam). **Operator RB or Y turn vacuum OFF.** It **never fires without a known position** (it can't check the G407 line), and stops the roller while climbing. Dashboard "Vacuum Mode".
- **Two tables**: SCORE (`ShotTable.defaults()` + "Shot Table Recorded") and PASS (`ShotTable.passDefaults()`, **starting guesses 3.5-13 m, hood high, RPM rising**, + "Pass Table Recorded"). "Record Shot Point" records into the table of the CURRENT target, so to tune passes: Start until "PASS only", Shot Tuning Mode ON, tune RPM (hood near the top, 6328's method) until the balls land on the corner point, Record. "Clear Recorded Pass Points" resets the pass table. Record passes on a real field (the shop ceiling is too low).
- **Shoot on the move** (`ShotSolver.solve`, the 2026 elite method): pose projected ahead by `kReleaseLatency` (0.10 s); the ball inherits the **turret pivot's** velocity (robot velocity + spin x the 0.16 m pivot offset); **virtual target** = target - pivot velocity x effective flight time, iterated until it moves < 1 cm (max 10); **drag**: lead by (1 - e^(-k t))/k with `kDragK` 0.42 (581 0.465, 6328 0.375; raw time over-leads ~20%); RPM/hood read at the virtual distance; **turret rate feedforward** (target sliding past + robot spin) through `Goober.aimAtAngleWithRate`. Dashboard "Aim Turret Rate (deg per s)", "Robot Speed (m per s)", Field widget objects Target and Aim Point.
- **Ready to fire** (per target): turret within atan(margin / distance) (0.15 m scoring, 0.50 m passing, never under 2 deg) AND turning at the target's rate within 30 deg/s (`Goober.aimTracking`; the old "slower than 30 deg/s" blocked real moving shots); robot speed under **1.5 m/s scoring / 4.0 m/s passing** (581's numbers); shooter within max(150 RPM, 4%) scoring / 400 RPM passing; hood within 0.12 / 0.25 rot. "Shot Ready" says what it's waiting for.
- **Hub shifts**: with FMS game data, a SCORING shot is held when the ball would land in our hub while it's CLOSED ("HUB CLOSED: opens in N s"): the hub is judged 0.3 s before the ball lands (so none lands while it's still closed; the match clock is approximate), and a ball still counts if it lands up to 2.5 s after the hub closes (rule: 3 s). Shooting starts early so the first balls land as it opens. **Passes are never held.** No game data (practice) = never held. Dashboard switch "Hold Fire: Hub Closed" (saved), "Our Hub".
- **Climb while shooting** (manual, no automation): while the elevator is above `Elevator.kRaisedThreshold` (5 rot), nothing automatic moves the robot or the intake: no heading assist, no agitation pumps or shimmy, vacuum won't deploy the intake. Shooting still works (legal: the tower is in our alliance zone, 3.4-4.0 m from the hub: record a shot point there). Dashboard "Climbing (elevator up)".
- **Trench guard**: the trench bar clears 22.25 in; the Limelight rides on the hood at 20.75 in (down) to 22.35 in (fully up). Near a trench (hub band +0.45 m + 0.4 s of travel, y within the opening +0.3 m) the hood target is capped at `kTrenchHoodMax` 3.0 rot. Dashboard "Near Trench (hood capped)". Manual hood jogs are not capped: keep the hood down through a trench.
- Elastic: new **Match** tab (Shot Target, Shot Ready, Our Hub, Vacuum Mode, Climbing, Near Trench, Pit Check, Field, distance, speed, RPMs, Agitator).
- **To tune on the robot**: `kReleaseLatency` (film a moving shot; balls land behind the motion = raise it), `kDragK` (balls land AHEAD in the direction of travel = lead too small = LOWER k), the pass table, and real flight times (film: release to landing; the recorded SCORE points all carry the 0.9 s placeholder "Tune TOF").

## Shuttle from their zone, pass-lane look-ahead, defense mode (Oct 27 evening)

- **Shuttle** (`TargetPicker.passTarget`, automatic in AUTO/PASS, vacuum too): a pass corner further than **"Pass Max Range (m)"** (dashboard, saved, default 12 m = 393 in; measure on a real field) is skipped. Deep in the OPPONENT's alliance zone no corner is in range, so it lobs to a **relay point in the neutral zone on our half**: 6.0 m (236 in) from our wall, 1.65 m (65 in) from the side wall, robot's side first (1678 did this). A partner or our own vacuum mode then passes it home. Dashboard "Shot Target" says SHUTTLE and "[in THEIR zone]". Behind their hub every lane is blocked: it holds ("move sideways").
- **Pass lane look-ahead**: a lane must miss both hubs (+0.30 m / 12 in) from the turret pivot NOW and from where it will be in `kLaneLookahead` 0.40 s (balls already in the uptake still leave), so feeding stops BEFORE a hub slides into the line and stays stopped while it's there; mid-volley it re-picks another clear lane if one exists. Covers both hubs (ours = G407 foul, theirs = points for them). Dashboard "Pass Lane Clear".
- **Defense mode** (`subsystems/DefenseMode`, **Driver Y** toggle): auto-aim and vacuum off, intake UP (`raiseIfDown`), hood stowed, stick slew 6/s (normal 3/s), sticks centered + robot under 0.3 m/s = wheels in an **X** (`SwerveDriveBrake`). Ends when the operator turns aim or vacuum on. Warnings (driver rumble + dashboard "Defense"): **G418 pin clock** (`defense/PinClock`: stick over 0.5, robot under 0.25 m/s, drive motors over 50 A average for 0.5 s; strong pulses from 2 s, rule is 3 s; resets after 3 s apart), and always in the last 30 s **G420** near THEIR tower (`defense/TowerGuard`, steady rumble). Numbers in `Constants.Defense`. The drive is already at its traction limit (120 A slip current x 4 ~ 1290 N vs ~800 N of grip), so more current would only spin the wheels.
- Rules checked (manual TU22): nothing forbids launching from the opponent's zone; G404 (not at robots), G405 (keep FUEL in the field), G415 (bumpers outside our zone: no damaging contact inside their frame), G418 (3 s pins), G420 (last 30 s, their tower).

## Drive Team Controller Map

`tools/controller-map/`: double-click **Start Controller Map.bat** on any Windows driver station laptop for a live map of both Xbox controllers under the docked Driver Station (reads the controllers in the background, so it follows presses while Driver Station has focus). See its README. **When a controller binding changes, update the `BIND` / `CALL` tables in `tools/controller-map/index.html` too.**

**Printable cheat sheet**: `docs/controls/Driver-and-Operator-Controls.pdf` (2 pages: the controller map picture + driver/operator tables, then the before-the-match checklist and how shooting works). Source: `docs/controls/controls.html` + `controller-map.png`. **When a binding changes, update its tables too**, re-screenshot the map, then open the HTML in Chrome and Print > Save as PDF (Landscape).

## Shoot From Anywhere (Oct 26)

**Testing it: follow `docs/LOCALIZATION-TESTING.md`** (written for absolute beginners). Dashboard "Localization Step" always says what to do next; "Aim Check" is the live PASS/FAIL; the "Test: Turret Sweep" button finds a backwards camera setting by itself (`LocalizationCheck`). Firing (ShootCommand) starts only when everything is ready, then keeps feeding through the RPM dip each ball causes (it used to pause after every ball).

The robot knows where it is on the field and aims at our hub from any spot, with or without a hub tag in view. Code: `frc.robot.aiming` (`ShotSolver` pure math, `ShotTable`), `subsystems/VisionLocalizer`, `subsystems/AimSubsystem`, `commands/AimCommand`, `commands/ShootCommand`. Tests: `src/test/java/frc/robot/aiming`.

- **Position**: swerve odometry + gyro, corrected by the Limelight seeing **any of the 32 field tags** (it no longer filters to hub tags; our hub tag is picked in code for the tag-only fallback). The camera rides on the turret, so `VisionLocalizer` tells the Limelight every loop where the camera is on the robot (`setCameraPose_RobotSpace`, from the turret angle). MegaTag1 (2+ tags) sets/corrects the heading; MegaTag2 (works with 1 tag) does the rest once the heading is known. Fixes are skipped while the turret turns faster than 120 deg/s or the robot spins faster than 2.5 rad/s, and single tags further than 4.5 m. The old MegaTag2 code in `CommandSwerveDrivetrain.periodic()` (wrong with a turret camera, raw Pigeon yaw) is removed.
- **Heading known** when: Driver LB with the robot pointing straight away from your driver station, an auto starts, or 2+ tags are seen. **Vision only sets the heading while it is unknown (after 10 steady 2-tag frames with the robot still, it SETS the whole pose with resetPose; a single blended fix only moved it part way); after that only the gyro (and Driver LB) changes it**, because field-centric driving uses the same heading (Oct 25: vision kept turning it after LB, so LB looked broken). **Position known** after the first vision fix or an auto start. Until both: "Pose Trusted" is false and auto-aim falls back to following our hub tag (last season's way).
- **Aim** (`AimCommand`, Operator RB toggle and autos): turret angle from robot pose + hub center (`Constants.Aim` hub coordinates, from the official 2026 welded field), measured from the turret pivot; position control with feedforward that cancels robot spin. Hood and shooter speed from the **shot table** by distance. **Shooting while moving**: aims at hub minus robot velocity x time of flight (`kShootWhileMoving`).
- **Dead zone** (the ~90 deg behind the turret's wires): the turret waits at the nearest end and the drive code turns the robot just enough (heading assist), only while the driver's rotate stick is centered.
- **Firing** (`ShootCommand`, Operator RT while auto-aim is on): starts feeding once READY (turret within 2 deg and slower than 30 deg/s, shooter within 150 RPM or 4%, hood within 0.12 rot), then keeps feeding. Turret has a 0.03 static-friction kick (`kTurretKS`) so it can close the last degrees. Dashboard "Shot Ready" says what it is waiting for, "Turret Aim Error (deg)", "Shooter RPM Error"; pauses by itself otherwise. Operator controller buzzes while ready. With auto-aim off, RT/LT work exactly as before.
- **Safety gate**: `Constants.Aim.kGeometryMeasured = false` turns vision position OFF and keeps auto-aim on the old tag-following behavior. **Set true Oct 25** with: turret pivot 6.475 in back-left of center (~5.3 back, 3.7 left), HOME = shooter facing the robot front, Limelight 5 in BEHIND the turret center facing the shooting direction, tilted 25 deg up (measured with the hood DOWN), turret range **300 deg** (measured Oct 25 with a square: with 280 the code read 84 at a true 90 and 169 at a true 180; re-measure the same way if the far end is ever re-set). **The Limelight rides on the hood**: lens height 20.75 in (hood down) to 22.35 in (hood up), so `VisionLocalizer` interpolates camera height by hood position every loop and skips vision while the hood moves. **Tilt stays 25 deg at any hood position** (measured Oct 25: a belt compensation mechanism holds the camera angle), so `kCameraPitchHoodUp` = `kCameraPitchHoodDown`. **Now 23** (Oct 25 evening the camera had tipped up and was re-aimed; phone level 22-24). The mount can shift: re-check the tilt if Aim Check starts failing.

- **Practice field switch** (dashboard "Practice Field: Hub Tags Only", saved in roboRIO Preferences): ON = the Limelight's tag filter is only our hub's 8 tags (red 2,3,4,5,8,9,10,11; blue 18,19,20,21,24,25,26,27). Needed on the shop field, whose climb (tags 15/16) and trench (7) are not at official positions (Oct 25: the pose jumped onto the hub). **OFF at competition.** "Re-find Position (camera)" forgets the pose so the next steady 2-tag view sets it again.

### Bring-up (in order)
1. **Measure** (robot on, turret at HOME) and type into `Constants.Aim`: turret pivot from robot center (`kTurretPivotX/Y`, m, forward/left +), shooter direction at HOME (`kHomeYawDegrees`, CCW from robot front: 90 = left side), camera lens from the pivot along the shooter direction / to its left / height (`kCameraForwardOfPivot`, `kCameraLeftOfPivot`, `kCameraHeight`), camera tilt up (`kCameraPitchDegrees`). Check the turret really covers `Turret.kEndDegrees` (300, measured Oct 25 with a square) at `kEndPosition` (36 rot). Then `kGeometryMeasured = true`.
2. **Limelight UI** (`http://limelight.local:5801`, laptop on the robot Wi-Fi): Configuration tab **Marker Size 165.1 mm** (was 339 until Oct 25, which doubled every distance); Advanced tab Full 3D on and **Field Map File = FRC2026_WELDED.fmap** (upload arrow next to "Field Map File", NOT the pipeline upload button at the top; Limelight downloads page). Limelight 4 runs LLOS 2025.0 (2026.1 available; update after Beach Blitz). The robot-space camera pose in the UI is overwritten by the code every loop.
3. **Camera direction test**: robot still, 2+ tags in view, watch the "Field" widget / "Vision Status". Jog ONLY the turret: the robot on the Field widget must stay put. If it rotates or swings: flip `kLimelightYawCCW`. If it mirrors left/right: flip `kLimelightSideIsRight`.
4. **Aim test**: from 4-5 spots, RB on: "Aim Turret Angle (deg)" vs where the shooter actually points; the shooter should point at the hub center. Then drive the hub into the dead zone and check the robot turns itself.
5. **Shot table** (no redeploys): turn on "Shot Tuning Mode", at each of 6-8 distances set "Tune RPM" / "Tune Hood (rot)" until shots go in, press **"Record Shot Point"** (saved on the roboRIO, Preferences key "Shot Table Recorded", used immediately; replaces points within 0.25 m). "Tune TOF (s)" = ball flight time, only for shooting while moving. Then copy "Shot Table" into `ShotTable.defaults()` so it survives a roboRIO reimage. "Clear Recorded Shot Points" goes back to the code's table.
6. Dashboard: "Field", "Vision Status", "Vision Tags", "Pose Trusted", "Heading Known", "Seconds Since Vision Fix", "Aim Distance (m)", "Aim Turret Angle (deg)", "Aim Hub In Reach", "Aim Robot Turn Needed (deg)", "Aim Shooter RPM", "Aim Hood (rot)", "Aim Mode" (POSITION / TAG ONLY), "Shot Ready" (why not), "Ready To Fire", "Aim Geometry", "Shot Table", "Turret Target (deg)".

## Vendor Dependencies

| Library | Purpose |
|---------|---------|
| Phoenix6 (26.1.1) | CTRE TalonFX motors, CANcoders, Pigeon2 IMU |
| PathPlannerLib (2026.1.2) | Autonomous path following |
| PhotonVision (photonlib) | USB camera AprilTag detection via PhotonCamera (unused: no USB cameras on the robot) |
| DogLog | Telemetry logging (NT publish, DS capture, PDH logging) |
| MapleSim (0.4.0-beta) | Physics simulation for swerve drivetrain |
| WPILib NewCommands | Command-based framework |

## Project Structure

```
src/main/java/frc/robot/
├── Main.java                  # Entry point
├── Robot.java                 # TimedRobot lifecycle, rumble feedback, DogLog init
├── RobotContainer.java        # Subsystem instantiation, bindings, auto chooser
├── Constants.java             # All CAN IDs, PID gains, speeds, positions
├── Marcos.java                # PathPlanner named command registration
├── HubShiftTracker.java       # REBUILT 2026 hub active/inactive shift logic
├── DriverDashboard.java       # Shuffleboard "Match Dash" tab (match phase, hub status, RPM)
├── Telemetry.java             # Swerve state → NetworkTables + SignalLogger
├── LimelightHelpers.java      # Limelight vendor helper (not directly used)
├── controls/
│   └── HotasX56               # Logitech X56 HOTAS wrapper (stick + throttle USB devices)
├── commands/
│   ├── FuelHandlingCommand     # Coordinated shooter + index + intake (fwd/rev)
│   ├── FeedShooterCommand      # Index + shooter intake only (shooter runs separately)
│   ├── RunShooterCommand       # Spin shooter at target RPM
│   ├── GoobaToggleCommand      # Deploy/stow the hood (named commands deployGooba/stowGooba)
│   ├── AutoGooba               # Hood follows the Limelight ty through the ty shot map (only the autoAimGooba named command now)
│   ├── AimCommand              # Shoot from anywhere: turret + hood from field position (tag fallback)
│   ├── ShootCommand            # Shoot from anywhere: table RPM, feeds only when ready
│   ├── VacuumCommand           # Operator Back: intake + auto-aim + fire-when-ready (score or pass)
│   ├── GooberAlign             # Old tag-only turret align w/ search sweep (not bound anymore: RB = AimCommand)
│   ├── MariosEarCommand        # Multi-phase turret aim (Limelight → USB cams) (RETIRED: no USB cams)
│   ├── ManualGoobaCommand      # Operator D-pad Up/Down jog of the hood
│   └── ManualTurretCommand     # D-pad jog turret left/right
├── subsystems/
│   ├── CommandSwerveDrivetrain # CTRE swerve + PathPlanner AutoBuilder + MapleSim
│   ├── VisionLocalizer         # Limelight field position (camera on the turret), MegaTag1/2 into odometry
│   ├── AimSubsystem            # Shot brain: turret angle, distance, shot table, ready-to-fire
│   ├── ShooterSubsystem        # Dual TalonFX (leader/follower), velocity control
│   ├── IndexSubsystem          # Single TalonFX ("Spindex"), feeds the shooter
│   ├── ShooterIntakeSubsystem  # Single TalonFX, feeds fuel to shooter
│   ├── GoobaSubsystem          # Hood, TalonFX 18, Motion Magic position + shot map
│   ├── Goober                  # Turret motor, duty cycle rotation, ~300° from HOME with soft limits
│   ├── LimelightSubsystem      # Limelight NetworkTables (tx/ty/ta/tv/botpose)
│   ├── MariosEar               # Multi-camera vision: Limelight + 2 PhotonVision USB cams (RETIRED, not created)
│   ├── GroundIntakePivotSubsystem # 2 TalonFX (20 leader, 21 follower): Motion Magic UP/DOWN toggle + jog
│   ├── ElevatorSubsystem       # Climber, TalonFX 19: hold-to-move up/down
│   └── IntakeRollerSubsystem   # Ground intake roller, TalonFX 22: hold Driver RT to spin
├── aiming/
│   ├── ShotSolver              # Pure aiming math (turret angle, dead zone, shoot on the move, camera pose)
│   ├── ShotTable               # Distance -> RPM / hood / time of flight (SCORE and PASS tables), recorded points
│   └── TargetPicker            # SCORE vs PASS by position (G407), pass corners, hub line-of-sight, trench zones
├── feeding/
│   ├── ShotDetector            # Counts balls from flywheel dips
│   ├── JamDetector             # Spindexer stall -> short reverse
│   └── AgitationPlanner        # When to pump the intake / shake the spindexer, volley stats
├── generated/
│   └── TunerConstants.java     # Tuner X generated swerve config (module IDs, offsets, gains)
└── sim/
    └── MapleSimSwerveDrivetrain # MapleSim physics simulation wrapper
```

## Robot Mechanisms

This is a 2026 FRC robot with the following mechanical systems:

### Swerve Drivetrain
- 4-module swerve drive using **Kraken X60** motors (TalonFX) + CANcoders
- Pigeon2 IMU (CAN ID 41) for heading
- Max speed: **4.39 m/s** at 12V
- Drive gear ratio: 7.03125, steer gear ratio: 26.09
- Wheel radius: 2 inches
- Robot footprint: ~29.5" x 24.5" (bumpers)
- Supports field-centric and robot-centric drive modes (Shuffleboard selectable)

### Shooter System
- **Bottom roller**: TalonFX CAN 16 + CAN 17 (both named "Shooter Bottom" in Tuner X), mounted mirrored.
  The code drives both at the same speed with opposite inversion, which is correct for a mirrored pair.
- **Top roller**: belted to the bottom roller, so it spins whenever the bottom roller does. **Nothing to code.** The Kraken X44 on CAN 18 (Tuner X name "Shooter Top") drives the **hood**, not this roller (programming lead, Oct 25).
- Velocity control (VelocityVoltage): fast 5400 RPM (`kMaxRPM` = 5800, the motor's free speed; was capped at 3100 until Oct 25), 0.25 s spin-up ramp, 60 A supply / 120 A stator per motor, slow 2500, idle 2000 (Driver D-pad Left), reverse -1000
- PID: **kP=0.6, kS=0.20, kV=0.15** (Oct 26 Shot Map Day: with kP 0.25 / kV 0.12 the wheel settled at ~82% of any target, so auto-aim never fired; measured LT at 11.1 V holding ~4450 RPM = ~0.15 V per rot/s; 0.18 overshot ~230 RPM). Dashboard "Shooter RPM Error"
- **Shot consistency meter** (`ShooterSubsystem.meterShots`): each ball dips the wheel; the speed just before the dip is the launch speed. Dashboard "Shots At Speed" (X of Y launched within 100 RPM of target), "Last Shot Launch RPM", "Last Shot Dip (RPM)", "Last Shot Recovery (s)", button "Reset Shot Stats"; DogLog Shooter/*. If later balls in a burst launch slow, the spindexer is feeding faster than the wheel recovers: lower "Spindex Forward Volts"

### Index / Feeder
- Single TalonFX (CAN 13, Tuner X: "Spindex"), VoltageOut (FOC off, unlicensed): +12 V forward by default, trimmed live with dashboard **"Spindex Forward Volts"** (0-12) for tuning after a gear change, -6 V reverse. Slow at 64:1 is the gearing: 64:1 = ~94 RPM, about 1 m/s at the 8 in omni rim. Recommended: **16:1** (two 4:1 MAXPlanetary stages, ~375 RPM, ~4 m/s free) and trim down with the dashboard volts if it flings or jams at the uptake; then lower the 160 A stator limit to ~80 A. Dashboard "Spindex Speed (rot/s)", "Spindex Amps", "Spindex Volts"
- Feeds fuel from intake to shooter, brake mode

### Shooter Intake
- Single TalonFX (CAN 15, Tuner X: "UpTakeChannelForShooter"). **Feeding uses VoltageOut at a fixed voltage** (`kForwardVolts` 11 V, dashboard **"Uptake Forward Volts"** tunes it live), not 100% duty: 100% meant 12.8 V on a fresh battery and ~11 V on a tired one, so the ball entered the shooter at a different speed each match. Reverse -0.6 duty. FOC off (unlicensed)
- **Feed order rule**: each stage a bit faster than the one before (spindexer rim < uptake < flywheel) so balls separate into single file. Tune Spindex and Uptake volts BEFORE recording the shot table; changing them later shifts every shot
- Pulls fuel into the shooter path, brake mode

### Gooba (Hood)
- **Kraken X44 on Talon FX 18** (Tuner X name "Shooter Top", no Pro license, so FOC off). Tilts the hood up/down to change the shot. Same code as last season, which ran it on ID 51. `Constants.Gooba.kHoodEnabled = false` turns it off entirely (motor never created).
- **Motion Magic** position control: kP=2.4, kD=0.1, cruise 80 rot/s, accel 160 rot/s². Brake, 40 A supply + 30 A stator limits.
- **0 = power-on position, so always power on with the hood PUSHED all the way DOWN by hand** (against the bottom stop). Left alone it springs back up about 0.4-0.5 rotations, so a hood that was just resting gives a different zero. No re-zero at startup (last season's code did). Dashboard button **"Zero Hood (hood is DOWN)"**. **Positive = UP** (flip `kInverted` if the jog Up lowers it).
- **Soft limits** `kBottomPosition` / `kTopPosition`: bottom **0.2** (measured Oct 25: pushed down = 0, rests at 0.42-0.51, the motor jogs it down to 0.24), top **3.8** (measured Oct 25: pushed all the way up = 3.89; last season's hood went to 10.7, so the gearing changed). To measure: power on with the hood DOWN, click **"Hood Limits OFF (measure)"** ("Hood Status" says LIMITS OFF), jog with Operator D-pad Up until the hood just touches the top and read "Hood Position (rot)", then D-pad Down to the bottom and read it again. Put a hair inside each number in `Constants.Gooba`, then click the switch again. The jog target never runs more than 0.5 rot ahead of the hood and is reset to the real position on release, so holding against an end doesn't keep pushing.
- **Two hood tables.** Shoot-from-anywhere (Operator RB, autos) sets the hood from the **distance shot table** in `AimSubsystem` (see "Shoot From Anywhere"). The **ty shot map** below is only used while the robot's position isn't known yet (AimCommand falls back to following our hub tag) and by the `autoAimGooba` named command.
- **ty shot map** keyed on the Limelight's **ty** (up/down angle to the tag): **a starting guess, not tuned yet**: last season's competition points scaled by 3.89/10.7 to this hood's travel: 27.3→0.36, -16.8→0.86, -18.3→1.38, -23.9→1.85 (last season: 1.0 / 2.36 / 3.8 / 5.1). They were recorded from raw tx on the old sideways mount (tx was the up/down angle then); the camera is in the same spot, so they carry over as ty. Close to the hub = big positive ty.
- **Controls**: Operator **RB** auto-aim moves the hood too (`AimCommand`: distance shot table, or the ty map in the tag fallback). Operator **Y** toggles stowed (0.2) / deployed (1.3, last season's 3.5 scaled). Operator **D-pad Up/Down** (hold) jogs it (2 rot/s). Any of these manual controls cancels auto-aim.
- **When the robot disables**, the hood target is dropped, so it never jumps on enable.
- Dashboard: "Hood Enabled", "Hood Status" (OK / At BOTTOM / At TOP / LIMITS OFF), "Hood Position (rot)", "Hood Target (rot)", "Hood Amps", "AutoGooba ty", "AutoGooba Target Rot".

### Goober (Turret)
- TalonFX (CAN 14, Tuner X: "AngleShooter"), duty cycle output (FOC off), brake mode, 30 A stator limit (low on purpose, to go easy on the wires).
- Turns the shooter **and the Limelight** (the camera rides on the turret). The Pigeon is on the frame, so turning the turret doesn't affect the gyro.
- **Range: about 300°, not 360°** (36 motor rotations; measured with a square Oct 25). The motor and Limelight wires run up to the turret, and they are the only thing that stops it. **HOME** = one END of the travel, lined up with the spot mark, shooter facing the robot front. From HOME it can only turn **RIGHT** (clockwise seen from above) about 300°, then back. The ~60° it can't reach (a bit more with the 10° aim margins) is the dead zone: with auto-aim on, the drive code turns the robot when the hub is in it (heading assist; the driver's rotate stick wins).
- **0 = power-on position, so always power on with the turret on the HOME mark.** The code does **not** re-zero at startup. Dashboard button **"Zero Turret (turret at HOME)"** (works while disabled). If it was powered on somewhere else: power off, turn it by hand to the mark, power on.
- **Positive = turning RIGHT = away from HOME.** If Operator D-pad Right turns it LEFT, flip `kMotorInverted`.
- **Soft limits**: HOME side at 0 (always on). Far side (300° from HOME) at the saved far end, on once it's set. **Set it from the dashboard**: power on at HOME, hold Operator D-pad Right until it's just short of where the wires get tight, press **"Set Turret Far End (turret at far end)"** (works while disabled; refuses if the turret is near HOME). It's saved on the roboRIO (WPILib Preferences, key "Turret End Position (rot)"), so it survives power-offs and deploys, and shows as "Turret Far End (rot)". **"Clear Turret Far End"** forgets it, to re-measure. `Constants.Turret.kEndPosition` is the backup used when nothing is saved: **36** (team decision Oct 24; the far end first read 32.49). **If neither is set**: NOTHING stops the turret past the far end ("Turret Status" warns), and auto-aim does nothing. `kEndDegrees` (**300**, measured Oct 25 with a square; was 270, then 280) turns rotations into degrees: re-measure it the same way if the far end is ever re-set.
- Auto-aim and the search sweep stay `kAimMarginDegrees` (10°) inside both ends. If the tag is further around than that, auto-aim stops and says TAG OUT OF REACH.
- Dashboard: "Turret Status" (setup / At HOME end / At FAR end / OK), "Turret Auto-Aim" (OFF / SEARCHING / TRACKING / LOCKED / TAG OUT OF REACH), "Turret Position (rot)", "Turret Angle (deg)" (0 = HOME), "Turret Far End (rot)", "Turret Amps".

### Ground Intake Pivot (raise/lower)
- **Leader**: TalonFX CAN 20 ("Right Ground Intake Raise"). The code only ever commands this one.
- **Follower**: TalonFX CAN 21 ("Left Ground Intake Raise"), set once at startup to copy the leader with `Follower(20, Opposed)` (mirrored mount). No soft limits on the follower.
- **Driver A toggles UP/DOWN** (FOC off). Pure toggle: it remembers the last press and alternates. When the code starts it checks the position once: below halfway counts as DOWN, so after a redeploy with the intake down the first press raises it (at power-on the motors read 0, so the first press goes DOWN). The A command is instant and schedules a fresh move, so every press counts even mid-move. Dashboard "Intake Next A Press" shows what the next press will do.
  - **DOWN**: Motion Magic down; once it has **arrived (close and stopped moving) or stalled on the floor**, it stops pushing and rests in brake (no humming). Waiting for arrival keeps both sides under motor control the whole way; stopping early let the intake free-fall and the sides drifted apart.
  - **UP**: Motion Magic up to `kUpHoldOffset` (0.5 rot) **below** UP = 0 and **keeps holding** there until the next press, so it can't drop mid-match (faint hum is normal). It never touches the top hard stop. Bigger offset = stops lower.
  - "Stall" = more than `kStallAmps` while slower than `kStallSpeed` for `kStallSeconds` (dashboard "Intake Stalled").
  - **ZERO WRONG**: if DOWN stops more than `kZeroWrongTolerance` short of the floor, "Intake Status" says `ZERO WRONG by X: push intake up, press Zero Intake`. Cleared by Zero Intake or the next clean DOWN.
- **Positive = UP.** No sensor knows the real angle (no shaft for a CANcoder; the mentor's rule is to always start with the intake up). UP = 0 comes from the Talon FX starting at 0 when the robot is **powered on**, so **push the intake all the way up against the top before every power-on**. It sags with the power off, and even a sag of a fraction of a rotation shifts every position: Oct 25 it powered on 0.9 low, DOWN slammed the floor and UP stopped too low to stay up (that was with the old let-go-at-UP code). The code does **not** re-zero at startup, so deploying new code keeps the true position. If the zero is ever wrong: push the intake up by hand and press Zero Intake. DOWN (`kDownPosition`) is a measured **negative** number. While it's 0.0, A does nothing and the dashboard says "Intake DOWN position not set". A stops `kDownBackoff` short of DOWN so the motors don't stall into the ground.
- **Operator Left Stick up/down** (hold) jogs it slowly (`kJogVolts`, VoltageOut, FOC off). It was on the D-pad until Oct 25; the D-pad is the hood now. The jog is a manual override: it **ignores the soft limits** while held (they come back on release), so it can always recover the intake if the zero is off.
- **Soft limit on the leader: DOWN only** (`kSoftLimitMargin` past DOWN, once DOWN is set). No UP soft limit: an UP limit measured from a wrong zero is what used to block raising.
- Brake mode, 40 A stator limit on both motors. Speed is set by `kCruiseVelocity` / `kAcceleration` / `kJerk`; holding by `kP` / `kD` / `kV`.
- **Enabling never moves it**: when the robot disables, the Motion Magic target is cleared, so it sits (brake) until the next A press or jog.
- Dashboard: "Intake Status", "Intake Position (rot)", "Intake Target (rot)", "Intake Holding Target", both motors' amps and speeds, plus a **"Zero Intake (intake is UP)"** button (works while disabled).
- Wrong direction: flip `kLeaderInverted`. Motors fighting: flip `kFollowerAlignment`. All in `Constants.GroundIntakePivot`.

### Elevator / Climber (Talon FX 19)
- Rises, latches on, pulls the robot up. **Hold Driver D-pad Up/Down** to move, release to stop. Positive = UP.
- `VoltageOut` (FOC off) at `kElevatorVolts` (2.0 V after a good first test at 1.5 V); direction via `kElevatorInverted`. Brake mode, 40 A stator limit.
- **Soft limits** (`kTopLimit` / `kBottomLimit`): the motor stops by itself at the top and bottom, even with the D-pad held, so it never stalls into the ends. **Measured Oct 24: `kTopLimit` = 125.1, `kBottomLimit` = 0.0, so the limits are ON** (the code turns them on whenever top > bottom). 0.0 is the power-on spot (fully down), so D-pad Down brings it all the way back to where it started; if it ever bangs or stalls at the bottom, use a small positive number instead (like 0.5). Dashboard "Elevator Status" shows OK / At TOP limit / At BOTTOM limit. Re-measure from "Elevator Position (rot)" if the elevator's gearing changes.
- **0 = power-on position, so power on with the elevator fully DOWN.** The code does **not** re-zero at startup (a redeploy with the elevator up would put the limits in the wrong place). Dashboard buttons: **"Zero Elevator (elevator is DOWN)"** and **"Elevator Limits OFF (recovery)"** (click to turn limits off, click again to turn them back on; for when it was powered on with the elevator up and a limit blocks the D-pad). No stall detection on purpose: during a climb the motor is slow and working hard, which looks like a stall, and cutting power there would drop the robot.
- The old climb code (motor 21) was deleted; the elevator replaced it.

### Ground Intake Roller (Talon FX 22)
- On CAN as ID 22 ("Intake motor(22)"), Pro-licensed. **Hold Driver Right Trigger** to spin it in at `kRollerVolts` (**12 V, full power** since Oct 25; was 9, then 6), **hold Driver Left Trigger** to spin it backwards at `kReverseVolts` (-9 V) and spit balls out. VoltageOut (FOC off), coast mode, 60 A stator limit (was 40).
- Spins balls out instead of in? Flip `Constants.IntakeRoller.kInverted`. `kIntakeRollerEnabled = false` turns it off entirely (motor never created).

### Vision
- **Limelight**: the robot's **only** camera. AprilTag detection, provides tx/ty/ta/tv/tid/botpose via NetworkTables (table name `limelight`). Model not confirmed (newer black finned body, no green LED ring; likely a 3G or 4): the web UI at `http://limelight.local:5801` shows it.
  - Mounted HORIZONTALLY and CENTERED on the shooter, **on the turret**, so it turns with it: tx aims the turret (tx > 0 = tag to the right), ty drives the distance trig
  - **The Limelight sees all 32 tags** (for field position, see "Shoot From Anywhere"; only our hub's 8 with the **Practice Field** switch ON); the tag-only fallback aim uses **only OUR hub tags**, picked in code from the raw tag list: the center tag of each of the 3 hub faces we shoot at. Red hub = 2, 5, 10; blue hub = 18, 21, 26 (checked against the WPILib 2026 field layout). Until the Driver Station reports the alliance it accepts both; after that only ours (dashboard "Limelight Aiming At"). For practice, set the team station (Red/Blue) in the Driver Station.
  - **Alliance matters**: it picks our hub (aim and tags), flips the autos for red, sets which way field-centric "forward" points, and drives the hub-open rumble. The aim code assumes **blue** until the Driver Station says otherwise. Practice: Driver Station "Team Station" dropdown. Competition: the FMS sets it.
  - Dashboard: "Limelight tx", "Limelight Tag ID", "Limelight Aiming At", "Limelight distance (m)"
  - Distance-to-target calculation using trigonometry (`distanceToTarget()`)
- **PhotonVision USB cameras / MariosEar (RETIRED, Oct 2026)**: there are no USB cameras on the robot. `MariosEar` and `MariosEarCommand` are kept but never created.

## Subsystem Responsibilities

| Subsystem | Hardware | Control Mode | Key Methods |
|-----------|----------|-------------|-------------|
| `CommandSwerveDrivetrain` | 8 TalonFX + 4 CANcoder + Pigeon2 | Field/Robot centric | `applyRequest()`, `seedFieldCentric()` |
| `ShooterSubsystem` | 2 TalonFX (leader/follower) | VelocityVoltage | `runAtRPM(rpm)`, `stop()` |
| `IndexSubsystem` | 1 TalonFX | VoltageOut (FOC off) | `run(volts)`, `stop()` |
| `ShooterIntakeSubsystem` | 1 TalonFX | DutyCycleOut (FOC off) | `run(speed)`, `stop()` |
| `GoobaSubsystem` (hood) | 1 TalonFX (18) | MotionMagicVoltage (FOC off) + soft limits | `setPosition(rot)`, `getPosition()`, `getRotationForTy(ty)`, `toggleCommand()`, `zeroCommand()` |
| `Goober` | 1 TalonFX (14) | DutyCycleOut (FOC off) + soft limits | `setMotorSpeed(pct)`, `aimAtTarget(tx)`, `zeroCommand()`, `stop()` |
| `LimelightSubsystem` | Limelight camera | NetworkTables | `getTX()`, `getTY()`, `isValidTarget()`, `distanceToTarget()` |
| `MariosEar` (RETIRED, not created) | none (no USB cameras) | PhotonCamera | `limelightHasTarget()`, `getLimelightTX()`, `getLeftResult()`, `getRightResult()` |
| `GroundIntakePivotSubsystem` | 2 TalonFX (20 leader, 21 follower) | MotionMagicVoltage + VoltageOut jog + Follower (FOC off) | `toggleCommand()`, `jogUpCommand()`, `jogDownCommand()`, `zeroCommand()`, `stopCommand()` |
| `ElevatorSubsystem` | 1 TalonFX (19) | VoltageOut (FOC off) + soft limits | `upCommand()`, `downCommand()`, `zeroCommand()`, `limitsOffCommand()`, `setVolts(v)`, `stop()` |
| `IntakeRollerSubsystem` | 1 TalonFX (22) | VoltageOut (FOC off) | `runCommand()`, `reverseCommand()` |

## Command Architecture

Commands follow the standard WPILib command-based pattern. Key patterns:

- **`FuelHandlingCommand`**: The core shooting/intake command. Coordinates index + shooterIntake + shooter simultaneously. `isForward=true` shoots, `isForward=false` reverses/intakes (-1000 RPM). Runs while held (autos add timeouts).
- **`FeedShooterCommand`**: Runs index + shooter intake only, so the shooter can be spun up independently (operator layout).
- **`MariosEarCommand`** (RETIRED, no USB cameras): Multi-phase turret aiming with priority: (1) Limelight PID tracking → (2) Left USB cam coarse search → (3) Right USB cam coarse search → (4) stop. Don't revive it as is: it swings at 100% power and ignores the turret's range.
- **`AimCommand`** (Operator RB toggle, `score`/`visionScore`/`aimTurret`/`alignTurret`): shoot-from-anywhere turret + hood aim from the field position; falls back to following our hub tag (turret on tx, hood on the ty map) while the position isn't known. **`ShootCommand`** (Operator RT with auto-aim on, autos): shooter at the table speed, feeds only when READY.
- **`GooberAlign`** (not bound anymore, kept): Limelight PID turret alignment on raw tx (camera is centered on the shooter): tx > 0 (tag on the right) turns the turret right. When no valid target is in view it sweeps the turret between just inside HOME and just inside the far end hunting for one. Does nothing until the far end is set. Never finishes on its own — bind with whileTrue or a timeout.
- **`AutoGooba`**: Continuous hood auto-aim that maps Limelight ty to hood position via the ty shot map, with a moving-average filter on ty. Only the `autoAimGooba` named command uses it now. Requires only the hood.
- **Instant commands**: `GoobaToggleCommand` finishes immediately after setting the hood target.
- **While-held commands**: `ManualTurretCommand`, the intake jog, elevator up/down and the roller (built with `startEnd`) run while held and stop on release.
- **Toggles**: Driver A flips the intake between UP and DOWN (`runOnce`). Operator RB turns `AimCommand` on/off with `Trigger.toggleOnTrue` (Operator A/X/LB backup shots turn the same command on). Operator Y flips the hood stowed/deployed.

## Controller Bindings

### Driver Controller (Port 0 - Xbox)
This matches the drive team's official map. **LOCKED** = practiced, must stay exactly like this.

| Input | Action | Map |
|-------|--------|-----|
| Left Stick | Swerve translate (cubic scaling) | LOCKED |
| Right Stick | Swerve rotate | LOCKED |
| Left Bumper | Reset gyro / seed field centric. **Robot pointing straight away from your driver station** (it also tells shoot-from-anywhere the heading) | LOCKED |
| Right Trigger (hold) | Ground intake roller (Talon FX 22) | LOCKED |
| Left Trigger (hold) | Ground intake roller **backwards** (spit balls out) | added Oct 25 |
| A (press) | Toggle ground intake DOWN / UP (does nothing until `kDownPosition` is set) | LOCKED |
| X | Reserved, bound to nothing (was the climb piston) | LOCKED |
| Y (press) | **DEFENSE MODE** toggle: intake up, hood down, quicker drive, X-lock when the sticks are centered, pin/tower warnings | added Oct 27 |
| D-Pad Up (hold) | Climb up (elevator 19) | LOCKED |
| D-Pad Down (hold) | Climb down (elevator 19) | LOCKED |
| D-Pad Left (press) | Toggle shooter idle at 2000 RPM (press again = off) | LOCKED |
| Start (hold) | Slow mode (button speed limiter) | LOCKED |

### Operator Controller (Port 1 - Xbox)
| Input | Action | Map |
|-------|--------|-----|
| Right Trigger (hold) | Shoot (Spindex + uptake). **Auto-aim on: spins to the table speed and fires only when ready** | LOCKED |
| Left Trigger (hold) | Shooter windup (auto-aim on: to the shot table speed for the distance) | LOCKED |
| Right Bumper (press) | Toggle auto-aim: **shoot from anywhere** (turret + hood from field position; falls back to following our hub tag until the position is known) | LOCKED |
| Y (press) | Toggle hood stowed / deployed ("Toggle Gooba Hood", as on the team map) | LOCKED |
| D-Pad Up/Down (hold) | Jog the hood up/down slowly (as on the team map) | OPEN |
| Left Stick up/down (hold) | Jog ground intake up/down slowly (manual backup; was the D-pad) | OPEN |
| D-Pad Left/Right (hold) | Manual turret jog (Right = turn right, away from HOME; diagonals count, so it doesn't stutter) | OPEN |
| B (hold) | Spindex backwards to clear a jam | not on map, kept on purpose |
| A (press) | **Backup shot: Midshot** (between hub and climb): seeds the pose there, auto-aim on, then RT | added Oct 25 |
| X (press) | **Backup shot: Left trench** (as seen from our DS) | added Oct 25 |
| Left Bumper (press) | **Backup shot: Right trench** | added Oct 25 (was unbound) |
| Back (press) | **VACUUM MODE** toggle: intake down + roller + auto-aim, fires by itself when ready (score in our zone, pass from the neutral zone) | added Oct 27 |
| Start (press) | **Shot target mode**: AUTO -> SCORE only -> PASS only | added Oct 27 |

Removed because they weren't on the map: Driver D-pad Right (tank-drive toggle, and tank mode itself) and the old Operator Left Bumper "aim while held" (LB is now the right-trench backup shot).

### HOTAS (Ports 2/3 - Logitech X56): SWITCHED OFF
`Constants.Hotas.kEnabled = false`, so only the two Xbox controllers can drive. The code stays in the
project for practice (set the flag true). Even when on, it can **never** use ports 0 or 1, and it
ignores an Xbox controller that lands in its slot. See `docs/X56-HOTAS-SETUP.md`.

### Speed Limiting
- **Global speed limiter**: Shuffleboard chooser (100%/75%/50%/25%), always active
- **Button speed limiter**: Shuffleboard chooser (50%/75%/25%/10%), active only while Start is held (Xbox only)
- **HOTAS throttle** (only when HOTAS is switched on): right throttle lever scales speed 15%-100% on top of the global limiter

### Rumble Feedback (driver controller, TELEOP only)
- Constant low rumble (0.1) while our HUB is open for scoring (`HubShiftTracker`)
- Pulsing rumble (0.9) during the last 5 seconds of each 25s SHIFT window

## PathPlanner Setup

### Configuration
- Holonomic mode, robot size 0.9m x 0.9m
- Default max velocity: 3.0 m/s, max acceleration: 3.0 m/s^2
- Default max angular velocity: 540 deg/s, max angular acceleration: 720 deg/s^2
- Robot mass: 74.088 kg, MOI: 6.883
- Drive motor: Kraken X60, wheel COF: 1.1

### AutoBuilder
- Configured in `CommandSwerveDrivetrain.configureAutoBuilder()`
- Translation PID: (10, 0, 0), Rotation PID: (7, 0, 0)
- Alliance-aware path flipping (Red vs Blue)
- Auto chooser defaults to "Tests"

### Named Commands (registered in `Marcos.java`)
The registered strings are referenced by the deployed `.auto`/`.path` files — do not
rename them without updating the autos in `src/main/deploy/pathplanner/`.

| Name | Command | Timeout |
|------|---------|---------|
| `spinUpShooter` | RunShooterCommand (fast RPM) | 1.5s |
| `spinUpShooterSlow` | RunShooterCommand (slow RPM) | 1.5s |
| `score` | AimCommand + ShootCommand: aim from position, fire every ball the moment it's ready | 3.5s |
| `scorePreload` | Spin up → FuelHandling forward → stop (no vision) | staged |
| `visionScore` | Same as `score` | 3.5s |
| `shoot` | FuelHandlingCommand (forward) | 2.0s |
| `stopShooter` | InstantCommand → shooter.stop() | - |
| `intake` | FuelHandlingCommand (reverse) | 2.0s |
| `aimTurret` | AimCommand (turret + hood from position) | 2.0s |
| `alignTurret` | AimCommand | 2.0s |
| `deployGooba` / `stowGooba` | Hood to deployed / stowed preset | instant |
| `autoAimGooba` | AutoGooba (hood follows ty) | 2.0s |
| `lowerIntake`, `togglePiston1`, `togglePiston2`, `runGroundIntake` | No-ops (pneumatics removed; kept so old autos load cleanly) | - |

### Paths & Autos
- **Autos**: `Center Preload - Dumb`, `Center Preload - Vision`, `Left 3-Piece Auto`, `Right 3-Piece Auto`
- **Paths**: see `src/main/deploy/pathplanner/paths/`

## CAN ID Map

Source of truth: **Phoenix Tuner X** (re-checked Oct 2026, all Talon FX on firmware 26.3.0.0).
For mechanisms, "Name" is exactly what the device is called in Tuner X, so you can match code to hardware.
For swerve it is the module position.

| ID | Device | Name | Code |
|----|--------|--------------|------|
| 1 | TalonFX | Back Right turn | Swerve (`TunerConstants`) |
| 2 | TalonFX | Front Right turn | Swerve |
| 3 | TalonFX | Back Left drive | Swerve |
| 4 | TalonFX | Back Right drive | Swerve |
| 5 | TalonFX | Front Left turn | Swerve |
| 6 | TalonFX | Front Left drive | Swerve |
| 7 | TalonFX | Back Left turn | Swerve |
| 8 | TalonFX | Front Right drive | Swerve |
| 9 | CANcoder | Front Right | Swerve |
| 10 | CANcoder | Front Left | Swerve |
| 11 | CANcoder | Back Left | Swerve |
| 12 | CANcoder | Back Right | Swerve |
| 13 | TalonFX | Spindex | `Constants.Index` |
| 14 | TalonFX | AngleShooter | `Constants.Turret` (Goober) |
| 15 | TalonFX | UpTakeChannelForShooter | `Constants.ShooterIntake` |
| 16 | TalonFX | Shooter Bottom | `Constants.Shooter.kLeaderId` |
| 17 | TalonFX | Shooter Bottom | `Constants.Shooter.kFollowerId` |
| 18 | TalonFX (Kraken X44) | Shooter Top | `Constants.Gooba` (**hood**; the name is old) |
| 19 | TalonFX | Elevator | `Constants.Elevator` (climber) |
| 20 | TalonFX | Right Ground Intake Raise | `Constants.GroundIntakePivot.kLeaderId` |
| 21 | TalonFX | Left Ground Intake Raise | `Constants.GroundIntakePivot.kFollowerId` |
| 22 | TalonFX (Pro) | Intake motor(22) | `Constants.IntakeRoller` (ground intake roller) |
| 41 | Pigeon2 | Pigeon 2 | Swerve |

All devices are on the roboRIO's built-in CAN bus (`TunerConstants.kCANBus` is `""`), no CANivore.
IDs 51 and 61 no longer exist (the hood moved from 51 to 18).
**Free IDs for new devices:** 23-40, 42-50, 52+. New CTRE devices ship as ID 0, so set the ID
in Phoenix Tuner X before writing code.

## Patterns & Conventions

- **Naming**: The team uses creative nicknames for subsystem classes: "Gooba" = hood, "Goober" = turret (easy to mix up with Gooba), "MariosEar" = multi-camera vision system (retired), "Marcos" = named-command registrar. Instance variables use plain descriptive names (`limelight`, `goober`, `groundIntakePivot`, ...).
- **Vision mounting**: The Limelight is mounted HORIZONTALLY and CENTERED on the shooter (2026 offseason remount) — `tx` aims the turret, `ty` feeds the distance trig. Re-measure `Constants.Limelight.kMountAngleDegrees`/`kLensHeightMeters` whenever the mount changes.
- **Hub shifts**: `HubShiftTracker.isOurHubOpen()` implements the REBUILT shift schedule (game data at teleop start + match timer). Driver rumble and the dashboards read it via `RobotContainer.isHubOpenForUs()`.
- **Constants**: All hardware IDs, PID gains, speeds, and positions live in `Constants.java` inner classes. Comments mark values as `//fixed` when confirmed on hardware.
- **Marcos.java**: Centralized PathPlanner named command registration, separate from RobotContainer. Named commands must be registered before `AutoBuilder.buildAutoChooser()`.
- **Motor control**: Phoenix 6 API throughout. TalonFX motors use `DutyCycleOut` for simple speed control, `VelocityVoltage` for RPM control (shooter), `MotionMagicVoltage` for smooth position control (retired hood), and `VoltageOut` + `Follower` for the two-motor intake pivot. A follower is set once at startup and must never be sent any other command.
- **Phoenix Pro licensing / FOC**: **only Talon FX 16, 17 (shooter bottom) and 22 (intake roller) have Phoenix Pro.** Phoenix 6 control requests turn FOC on by default, and FOC is a paid Pro feature: an unlicensed Talon FX refuses FOC commands and won't move (LED alternates red/green). **Every control request for any other motor must use `.withEnableFOC(false)`**. All current code does.
- **Simulation**: MapleSim-backed swerve physics simulation with 0.002s loop period. Simulated robot: 115 lbs, KrakenX60 motors, 1.2 tire COF.
- **Logging**: DogLog for structured logging (battery, odometry, module states). Telemetry class publishes swerve state to NetworkTables + SignalLogger. HootAutoReplay for timestamp/joystick replay.
- **Drive input**: Cubic scaling on joystick axes (`Math.pow(abs, 3) * signum`) for fine control at low speeds.
- **TunerConstants**: Auto-generated by CTRE Tuner X, contains all swerve module configurations. Do not manually edit without re-running Tuner X.

## Known TODOs / FIXMEs

- **Talon FX 22 replaced Oct 27** with a used Kraken. It still dragged the bus down with both CAN pairs connected (fine on one pair): its pass-through is damaged, so it is wired as a short stub (one pair, the other capped). Watch Driver Station Diagnostics "CAN Bus Off"; replace the motor when possible.
- **Talon FX 22 (intake roller) took down the whole CAN bus, Oct 26 night**: every device blinked red ("no CAN"); with 22 unplugged everything else went orange. Bus wiring measured fine (121 ohm yellow-to-green at the roboRIO plug, roboRIO unplugged). Next: test 22 ALONE on a short known-good CAN run to the roboRIO. Orange = its CAN leads/robot wiring are damaged (check where they cross the intake pivot, which flexed a lot with agitation pumps): splice. Still red = its CAN transceiver is dead: replace the Kraken (new one ships as ID 0: set ID 22 + name "Intake motor(22)" in Tuner X; it won't have Pro, which the roller doesn't need). Until fixed, 22 can stay unplugged: the roller and agitation roller blinks just do nothing, Pit Check lists it.
- **Shot table (Oct 26)**: recorded on the robot at 1.5, 2.3 (Midshot), 3.0, 3.55 (Trench) m; 4.5 and 5.25 m still to do (shop ceiling blocks long arcs: record at Beach Blitz). The points live in roboRIO Preferences "Shot Table Recorded" until pasted into `ShotTable.defaults()` from the "Shot Table Code" dashboard line / http://10.56.69.2:5805/shot-log.csv.

- **Elevator**: soft limits are measured and on (`kTopLimit` = 125.1, `kBottomLimit` = 0.0, Oct 24). Climbing the full robot may need more than 40 A stator; now that the limits are in, it's OK to raise `kStatorCurrentLimit` in small steps if a real climb stalls
- **Intake DOWN position**: `kDownPosition = -5.7`, measured Oct 23 (floor: right -5.73, left -5.83). Re-measure if the intake or its gearing changes
- **Intake roller**: confirm spin direction (`kInverted`) and speed (`kRollerVolts`) on the robot
- **Intake pivot sync**: Oct 23 Elastic data showed the two sides within ~0.03 rot while driven and ~0.1 rot at rest, so they're well coupled; the drift came from an unpowered free-fall after stopping early (fixed). Dual position control isn't needed unless that changes
- **Turret**: done Oct 25: far end 36 rotations = 300° (`kEndPosition` / `kEndDegrees`), D-pad Right turns it right, and auto-aim turns toward the hub (Aim Check PASS). Re-measure `kEndDegrees` with a square if the far end is ever re-set
- **Shoot from anywhere**: verified on the robot Oct 25 on the shop field (Aim Check PASS, Turret Sweep OK-ISH 11 cm, aim test good with Practice Field ON). **Still to do: the shot table** (points are starting guesses). **Before competition: Practice Field switch OFF, re-check Aim Check on the real field.** Clear the roboRIO disk (~5 MB free) so DogLog can save
- Re-measure `Constants.Limelight.kMountAngleDegrees` and `kLensHeightMeters` on the new camera mount — `distanceToTarget()` is wrong until then
- Oct 25: fast raised to 5400 RPM (cap 5800 = motor free speed); 3100 could not reach the target. Kept below free speed so the loop regulates shot speed
- X56 HOTAS is switched off; if re-enabled, its axis/button indices need on-hardware verification (docs/X56-HOTAS-SETUP.md)
- Servo mechanism(s) replacing the pneumatics: not built yet, not coded yet (use WPILib `Servo` on a roboRIO PWM port when ready)
- **Hood bring-up (Oct 25)**: confirm Operator D-pad Up raises it (else flip `Gooba.kInverted`), limits measured (0.2 / 3.8); **re-record the shot map** with real shots (the current points are last season's, scaled)
