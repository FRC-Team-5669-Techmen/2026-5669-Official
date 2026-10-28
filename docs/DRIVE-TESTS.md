# Drive tests: make the robot drive where the path says

Three dashboard buttons measure the numbers the drivetrain needs. Each one shows its result on
**"Drive Test"** (Elastic tab **Drive Tests**) and prints the exact code change in the Driver
Station console as `DRIVE TEST | ...`.

**Before every test**
- Fresh battery (12.6 V or more at rest).
- Robot **enabled in TeleOperated**, everyone back, someone's hand on **Enter** (disable).
- Touching the driver's sticks stops a test at once. Clicking its button again stops it too.

## 1. Test: Wheel Radius (do this first)

Why: the code uses the catalog wheel size, 2.000 in (50.8 mm). Worn tread is smaller, so the robot
thinks it went further than it did, and every path ends short by the same percent.

1. Put the robot on the field carpet with about 1 m (3 ft) of clear space all around.
2. Click **Test: Wheel Radius**. The robot speeds up slowly and spins in place about 2 turns
   (about 20 s), then stops by itself.
3. Write down the number. Run it 3 times and average.
4. If it is more than 0.01 in (0.25 mm) away from 2.000, change `kWheelRadius` in
   `generated/TunerConstants.java` and **Wheel Radius** in the PathPlanner app's Settings to the average.

Re-run it whenever the tread is replaced or after a long event.

## 2. Test: Slip Current

Why: above this current the wheels only spin. Every spin loses traction and makes the robot think
it moved when it didn't. The code now allows 70 A (it was 120 A); a robot this size usually slips around 60 to 80 A.

1. Drive the robot so its **front bumper is flat against a wall** (the field wall or a solid shop wall).
2. Click **Test: Slip Current**. It pushes harder and harder (up to about 16 s) until the
   wheels break loose, then stops.
3. Run it 3 times. Use the **lowest** number.
4. Put it in `kSlipCurrent` (`generated/TunerConstants.java`) and **Drive Current Limit** in the
   PathPlanner app's Settings.

If it says the robot MOVED, the bumper wasn't flat against something solid.

## 3. Test: Drive Feedforward

Why: tells the drive motors how many volts each speed needs (kV) and how many volts it takes to
start moving (kS). The code has kS = 0, so slow, fine moves (the end of every path) lag.

1. Clear **2.5 m (8 ft)** of carpet in FRONT of the robot.
2. Click **Test: Drive Feedforward**. The robot creeps forward faster and faster up to about
   1 m/s (3 ft/s), then stops (about 7 s).
3. Run it 2 times. Put the numbers in `driveGains` in `generated/TunerConstants.java`
   (`.withKS(...)` and `.withKV(...)`).

## After the tests

- **Weigh the robot** with bumpers and battery. Put the weight (kg) in the PathPlanner app's
  Settings, **Robot Mass**. PathPlanner pushes the robot with that number.
- Redeploy, run the auto 3 times, and compare where it stops with where the path ends.
