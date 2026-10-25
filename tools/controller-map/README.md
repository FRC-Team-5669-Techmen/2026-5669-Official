# Techmen 5669 Controller Map

A live map of both Xbox controllers for the drive team. It sits under the docked Driver
Station and lights up every button as you press it while you drive, with a small robot
model showing what each button does.

## Use it (any Windows driver station laptop)

1. Get this repo on the laptop (clone it, or GitHub > Code > Download ZIP and unzip).
2. Plug in both Xbox controllers. Driver Station's **Joysticks** light should be green.
3. Dock Driver Station (its **Window** button) so it sits across the top of the screen.
4. Open `tools/controller-map` and double-click **Start Controller Map.bat**.

The map opens in its own window under Driver Station. Keep clicking and driving in Driver
Station as usual; the map follows the controllers in the background.

- Driver and operator swapped? Click **Swap pads** on the map.
- To stop it: close the map window, then close the minimized "Techmen 5669 Controller Map"
  window on the taskbar.
- Works offline (robot Wi-Fi is fine). Nothing is sent anywhere: the map runs only on the
  laptop at `127.0.0.1:5669`.
- Needs Chrome or Edge (every Windows laptop has Edge). No install, no admin.
- From a downloaded ZIP, Windows may warn the first time: click **Run** (or More info > Run anyway).

## "Get an app to open this 'ms-gamebar' link" popup

Windows shows this when a controller is plugged in or its Xbox button is pressed and Game
Bar isn't installed. Double-click **Stop Game Bar popup.reg** once per laptop (current user
only, no admin). **Undo Game Bar popup fix.reg** puts it back.

## If it doesn't react

The chip at the top right of the map reads like `31/s · 34 reads/s · XI 0,0 · JOY 1,0`:

- First two numbers near 30: the map is running.
- `XI` / `JOY` values change when you press buttons: the laptop sees the controllers.
  If they stay `-` or `0`, Windows isn't seeing the controllers (check the Joysticks light).
- `controller-map.log` in this folder records each start (which controllers were found).

## Keeping it accurate

The button labels come from `RobotContainer.java`. When a binding changes, update the
`BIND` and `CALL` tables near the bottom of `index.html` (search for `const BIND`).
Clicking a button on the map also works with no controllers, for teaching the layout.
