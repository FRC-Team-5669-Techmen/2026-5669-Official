# Practice matches (full timed matches, no FMS)

The robot runs the match; a second device shows it like the field does.

## Run one

1. **Driver Station laptop**: team station **Red 1**. Mode **Practice**. In the Setup tab, set the
   practice timing to **Autonomous 20** and the rest so the match runs at least **2:20 of teleop**
   (the Driver Station's box names and defaults weren't confirmed; the robot keeps its own REBUILT
   clock anyway, see below). Pick the auto in Elastic. Enable.
2. **Match screen** on a second laptop, a TV or a phone joined to the robot Wi-Fi:
   **http://10.56.69.2:5806** (USB tether: http://172.22.11.2:5806), or double-click
   `tools/practice-match/Start Practice Match.bat`. Keep Elastic on the driver station.
3. **Scorer**: one student taps **+1** on the match screen for every ball that goes IN our hub, and
   the tower level at the end. The robot counts what it shot by itself.
4. After the match, the row is saved on the screen's device (Export CSV) and on the robot:
   http://10.56.69.2:5805/practice-matches.csv.

## What happens

- **Auto** runs exactly as in a match.
- **Teleop**: when teleop comes right after an auto (Practice mode; the manual has a 3 s gap, the field
  measured about 4 s), the robot treats it as a match: it keeps its own clock (teleop 2:20) and
  **picks the auto winner at random** (switch "Practice: Random Auto Winner", saved, ON). Everything
  that depends on the hub schedule now behaves like a real match: shift rumbles, the hub-closed
  hold, "Our Hub". Plain teleop (plotting, drive tests) never gets a clock or a random winner.
- **Match screen**: phase (AUTO, TRANSITION, SHIFT 1-4, END GAME), seconds left, match clock, both hub
  lights (field behavior from manual 5.4: an active hub PULSES for its last 3 s; in the transition
  a white chase marks the hub that is off in shift 1), "opens/closes in N s", the auto winner, a
  timeline of our hub, synthesized cues in the manual's order (start, auto end, 3 bells, shift
  pops, end-game whistle, end buzzer), optional **coach voice** ("we won auto, closed first", "ten
  to open", "five, four...", "climb now" at 0:20), and the scorecard: auto balls, balls shot into
  our hub while it will be OPEN when they land, while CLOSED (wasted), passes, climb lift/hang
  time, battery.
- **Standalone start** (or Space) runs the same timeline on the device alone (3 s countdown, auto 20,
  3 s pause, teleop 2:20, random winner): coach drills, walk-throughs, sim sessions.

## What to track (drive team progress)

Per match: balls in (scorer), balls shot while closed (should go to 0), auto balls, hang time.
Teams that tracked fuel per active period and solo points per match saw clear gains (4590: 316 to
462 solo points); 5000 said never running a full practice match before their first event hurt.

## Other option: Cheesy Arena

Team 254's open-source FMS (`main`, 2026) runs REBUILT with real timing, sends random game data
(no hub sensors = auto tie = random winner) and has audience/wall displays. It needs the laptop at
10.0.100.5 and the radio set up for it: more setup than this, and it doesn't count our shots.

Sources: REBUILT Game Manual TU22 sections 5.4 (hub lights), 5.12 (sounds), 6.4 (timeline,
game data); WPILib DriverStation docs (match time is -1 without FMS outside Practice mode; DS Game
Data reaches the robot only at teleop enable or the practice End Game time); Cheesy Arena
(github.com/Team254/cheesy-arena); Chief Delphi REBUILT practice threads. Research notes:
/home/claude/research_notes/Practice match simulator/.
