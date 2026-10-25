# Testing Shoot-From-Anywhere (for absolute beginners)

You do not need to know anything about localization to do this. Follow the steps in order.
Each step says what it proves, what "good" looks like, and what to do if it's bad.
**Always have two people at the robot. A mentor never runs the robot alone.**

## The idea in 30 seconds

- The robot keeps track of **where it is on the field** (x, y) and **which way it faces**.
  That's called its *pose*. The wheels estimate it (like counting steps), and the
  **Limelight fixes it** every time it sees AprilTags (the square black-and-white stickers
  around the field; the code knows exactly where every tag is).
- Once the pose is right, the code knows where our hub is, so it can aim the turret and pick
  the shooter speed **from anywhere**, even when no tag is in view.
- Our Limelight is on the turret and the hood, so it moves. The code tells the Limelight where
  it is on the robot every loop. If that math has a sign backwards, the pose comes out wrong.
  **The tests below catch that for you and tell you which setting to flip.**

## Where to look

Open Elastic (or Shuffleboard) and find these. Everything is under the normal SmartDashboard list.

| Name | What it means |
|------|---------------|
| **Localization Step** | One sentence: what to do next. Start here. Always. |
| Vision Status | What the Limelight is doing this instant ("OK MT1 2 tags", "no tags", "skip: turret turning fast"...) |
| Vision Tags | How many tags it sees right now |
| Heading Known / Pose Trusted | Green = the robot knows which way it faces / where it is |
| **Aim Check** | PASS, or "OFF BY x deg". The single best "is it working" number |
| Aim Check (deg) | The same number. -999 = no hub tag in view |
| **Turret Sweep Result** | Result of the automatic turret test |
| Field | A field picture with the robot, our hub and the aim point drawn on it |
| Aim Distance (m) | How far the code thinks the hub is |

Everything is also **logged**: the Driver Station console prints lines starting with
`LOCALIZATION |`, and DogLog saves `Localization/*`, `Vision/*` and `Aim/*` to the log file.
(The roboRIO disk was nearly full on Oct 25. Clear old logs in the roboRIO's web page or DogLog
can't save.)

## Before every test

1. Robot powered on with the **turret on its HOME mark**, intake UP, elevator DOWN.
2. Turret far end is set (if "Localization Step" says "Set the turret far end first", ask the
   coder; it's the "Set Turret Far End" button with the turret at its far end).
3. Driver Station shows the right alliance color (Team Station Red 1 / Blue 1). The code aims at
   **that alliance's** hub.

## Step 1: Does it see tags? (robot DISABLED, push it by hand)

Point the Limelight at tags 1 to 4 meters away.

- Good: **Vision Tags** is 1 or more; **Vision Status** says "OK ...".
- Bad: "no tags". Check the Limelight is powered (green light), the field tags are there, and in
  the Limelight web page (http://limelight.local:5801, laptop on the robot Wi-Fi) the pipeline is
  the AprilTag one, **Marker Size 165.1** (Configuration tab), **Full 3D** on and **Field Map File
  FRC2026_WELDED.fmap** (Advanced tab, the upload arrow next to "Field Map File").

## Step 2: Does it know where it is? (still DISABLED)

Easiest: put the robot 2-3 m in front of a hub face so it sees **both tags on that face**, and
hold it still. "Vision Status" counts "heading: steady 1/10 ... 10/10", then says "OK SET from
camera". No button needed. (Or press **Driver LB** with the robot pointing straight away from
your driver station wall; on the red side that means pointing at the hub face with tags 9 and 10.)

- Good: **Heading Known** and **Pose Trusted** turn green. On the **Field** picture, the robot
  is roughly where it really is on the field and facing the right way.
- Bad: the robot on the picture is somewhere silly. Go to Step 4 (Turret Sweep): it finds the
  cause.

## Step 3: The Aim Check (still DISABLED, the most important test)

Push the robot around so the Limelight sees **our hub's** tags. Try 3 or 4 different spots and
turn the turret by hand to different angles.

- What it does: compares where the robot *thinks* the hub tag is (from its pose) with where the
  camera *actually* sees it.
- Good: **Aim Check** says **PASS** (within 3 degrees) nearly everywhere. "Aim Check Tally"
  shows mostly good loops.
- Bad: "OFF BY 10 deg" or more. Do Step 4.

## Step 4: Test: Turret Sweep (robot ENABLED, one person on the disable button)

The robot must sit **still** (don't drive), seeing tags 1 to 4 meters away (2+ tags is best).
Clear people away from the turret.

1. Enable in Teleop.
2. Click the **Test: Turret Sweep** button on the dashboard.
3. The turret turns slowly from one end to the other and back (about 40 seconds). Don't touch
   the controllers.
4. Read **Turret Sweep Result** (also printed in the Driver Station console).

| Result says | Do this |
|-------------|---------|
| PASS | Done. Go to Step 5. |
| OK-ISH | Fine for now. Tell the coder "small wobble". |
| FAIL: camera turns the wrong way | Coder flips `Constants.Aim.kLimelightYawCCW` (true to false or false to true), deploys, run it again |
| FAIL: position moves as the turret turns | Coder flips `Constants.Aim.kLimelightSideIsRight`, deploys, run again. Still failing: re-measure the camera offset |
| NOT ENOUGH TAG VIEWS | Move the robot so the camera sees 2 tags 1 to 4 m away and run again |

Disable the robot when it finishes.

## Step 5: Aim test (ENABLED, shooter OFF, no balls)

1. Operator **RB** turns auto-aim on.
2. Push or slowly drive the robot to different spots. The turret should keep pointing at our hub,
   even when the hub tags are out of view.
3. Put the hub behind the robot: the robot itself turns so the turret can reach (that's normal).
4. **Shot Ready** on the dashboard says what it's waiting for.

Good: the turret stays on the hub everywhere. Bad: tell the coder which spot and what "Aim Check"
said there.

## Step 6: Shots (ENABLED, balls, safety glasses)

1. Auto-aim on (Operator RB), hold **Operator RT**. It only starts feeding when everything is
   ready, then keeps feeding until you let go.
2. Misses short or long? That's the **shot table**, not localization. Turn on "Shot Tuning Mode",
   change "Tune RPM" / "Tune Hood (rot)" until it goes in, press **Record Shot Point**. Do 4 or 5
   distances. It's saved on the robot right away; no redeploy.

## If anything is weird

- Switch **Vision Pose ON** off: the robot stops using the camera for position, and auto-aim
  goes back to simply following the hub tag (the old way).
- Write down what "Localization Step", "Vision Status" and "Aim Check" said. Those three lines
  are enough for the coder (or Claude) to figure it out.
