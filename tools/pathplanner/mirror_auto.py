"""
Mirror a PathPlanner auto to the OTHER SIDE of our alliance (left <-> right), paths and all.

PathPlanner already flips every auto from blue to red by itself. This is the other thing: it makes
the Left version of a Right auto (or the reverse). The field is symmetric about the line through
both hubs' centers (y = 4.0346 m), so every y becomes 2 * 4.0346 - y and every angle changes sign.

Usage (from the project folder; PathPlanner closed or it may overwrite the files):
    python tools/pathplanner/mirror_auto.py "Right 3-Piece Auto" Right Left

Every path in the auto gets a mirrored copy with "Right" replaced by "Left" in its name (a name
without "Right" gets " (Left)" added), and so do linked waypoint names and folders. The new auto is
the auto's name with the word swapped. Files that already exist are OVERWRITTEN (git keeps the old
ones). Then it checks every shooting spot of the new auto (the path right before each score).
"""
import json
import math
import os
import sys

MIRROR_Y = 4.0346          # hub center line (Constants.Aim.kHubY)
HUB = (4.6255, 4.0346)     # our (blue) hub center
SCORE_LINE_X = 4.19        # G407: robot center must be within this of our wall to score
CENTER_LINE_X = 8.27       # G403: never FULLY cross it in auto
HALF_LENGTH = 0.375        # 29.5 in bumpers / 2
HALF_WIDTH = 0.311         # 24.5 in bumpers / 2 (the robot's shortest reach behind its center)
TURRET_REACH = (10.0, 290.0)  # turret degrees it can aim (HOME = robot front, turning right)
SCORE_COMMANDS = {"score", "visionScore", "scorePreload"}

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                    "src", "main", "deploy", "pathplanner")


def swap(name, a, b):
    if a in name:
        return name.replace(a, b)
    return f"{name} ({b})"


def mirror_point(p):
    return None if p is None else {"x": p["x"], "y": 2 * MIRROR_Y - p["y"]}


def mirror_angle(deg):
    return 0.0 if deg is None else -deg


def mirror_path(d, a, b):
    d = json.loads(json.dumps(d))
    for w in d["waypoints"]:
        w["anchor"] = mirror_point(w["anchor"])
        w["prevControl"] = mirror_point(w.get("prevControl"))
        w["nextControl"] = mirror_point(w.get("nextControl"))
        if w.get("linkedName"):
            w["linkedName"] = swap(w["linkedName"], a, b)
    for r in d.get("rotationTargets", []):
        r["rotationDegrees"] = mirror_angle(r["rotationDegrees"])
    for z in d.get("pointTowardsZones", []):
        z["fieldPosition"] = mirror_point(z["fieldPosition"])
        z["rotationOffset"] = mirror_angle(z.get("rotationOffset", 0.0))
    for key in ("goalEndState", "idealStartingState"):
        if d.get(key):
            d[key]["rotation"] = mirror_angle(d[key]["rotation"])
    if d.get("folder") and a in d["folder"]:
        d["folder"] = d["folder"].replace(a, b)
    return d


def mirror_commands(cmd, names):
    t, data = cmd["type"], cmd.get("data", {})
    if t == "path":
        data["pathName"] = names[data["pathName"]]
    elif t in ("sequential", "parallel", "deadline", "race"):
        for c in data["commands"]:
            mirror_commands(c, names)


def path_names(cmd, out):
    t, data = cmd["type"], cmd.get("data", {})
    if t == "path":
        out.append(data["pathName"])
    elif t in ("sequential", "parallel", "deadline", "race"):
        for c in data["commands"]:
            path_names(c, out)
    return out


def flat(cmd, out):
    t, data = cmd["type"], cmd.get("data", {})
    if t in ("sequential", "parallel", "deadline", "race"):
        for c in data["commands"]:
            flat(c, out)
    else:
        out.append((t, data.get("pathName") or data.get("name")))
    return out


def bezier_xs(d):
    w = d["waypoints"]
    xs = []
    for p, q in zip(w, w[1:]):
        pts = [p["anchor"], p["nextControl"], q["prevControl"], q["anchor"]]
        for i in range(41):
            t = i / 40
            u = 1 - t
            xs.append(u**3 * pts[0]["x"] + 3 * u * u * t * pts[1]["x"]
                      + 3 * u * t * t * pts[2]["x"] + t**3 * pts[3]["x"])
    return xs


def check(auto, paths):
    print("\nChecks for the new auto:")
    ok = True
    steps = flat(auto["command"], [])
    for i, (t, name) in enumerate(steps):
        if t == "path":
            deepest = max(bezier_xs(paths[name]))
            behind = CENTER_LINE_X - (deepest - HALF_WIDTH)  # worst case: robot sideways
            if behind <= 0:
                ok = False
                print(f"  !! {name}: robot center reaches x {deepest:.2f} m, the WHOLE robot can be "
                      f"past the center line (G403)")
            elif deepest + HALF_LENGTH > CENTER_LINE_X:
                print(f"  {name}: partly over the center line, back edge still {behind:.2f} m on our "
                      f"side (G403 = never FULLY cross; leave room for drift)")
        if t == "named" and name in SCORE_COMMANDS:
            prev = next((n for tt, n in reversed(steps[:i]) if tt == "path"), None)
            if prev is None:
                continue
            d = paths[prev]
            end = d["waypoints"][-1]["anchor"]
            heading = d["goalEndState"]["rotation"]
            dist = math.hypot(HUB[0] - end["x"], HUB[1] - end["y"])
            bearing = math.degrees(math.atan2(HUB[1] - end["y"], HUB[0] - end["x"]))
            turret = (-(bearing - heading)) % 360.0
            legal = end["x"] < SCORE_LINE_X
            reach = TURRET_REACH[0] <= turret <= TURRET_REACH[1]
            ok &= legal and reach
            print(f"  {name} after {prev}: at ({end['x']:.2f}, {end['y']:.2f}) facing {heading:.0f} deg, "
                  f"{dist:.2f} m from the hub, turret {turret:.0f} deg"
                  f"{'' if legal else '  !! OUTSIDE the scoring line (G407)'}"
                  f"{'' if reach else '  !! hub in the turret DEAD ZONE'}")
    print("  all good" if ok else "  FIX the lines marked !!")


def sync_linked_elsewhere(new_paths):
    """
    A linked waypoint can also live in a path that isn't in the auto (e.g. LeftStart.path).
    PathPlanner keeps linked waypoints on the same spot when it opens the project, so a stale copy
    would drag the new path back to the old spot (Oct 29: Left-to-Pickup jumped back to the old
    start). Move those copies, with their control points, onto the new spot.
    """
    spots = {}
    for d in new_paths.values():
        for w in d["waypoints"]:
            if w.get("linkedName"):
                spots[w["linkedName"]] = w["anchor"]
    folder = os.path.join(ROOT, "paths")
    for f in sorted(os.listdir(folder)):
        name = f[:-len(".path")]
        if not f.endswith(".path") or name in new_paths:
            continue
        with open(os.path.join(folder, f)) as fh:
            d = json.load(fh)
        moved = False
        for w in d["waypoints"]:
            spot = spots.get(w.get("linkedName"))
            if spot is None or (w["anchor"]["x"], w["anchor"]["y"]) == (spot["x"], spot["y"]):
                continue
            dx, dy = spot["x"] - w["anchor"]["x"], spot["y"] - w["anchor"]["y"]
            for k in ("anchor", "prevControl", "nextControl"):
                if w.get(k):
                    w[k] = {"x": w[k]["x"] + dx, "y": w[k]["y"] + dy}
            moved = True
        if moved:
            with open(os.path.join(folder, f), "w") as fh:
                json.dump(d, fh, indent=2)
            print(f"path  {name} (linked waypoint moved to match)")


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    auto_name, a, b = sys.argv[1:]
    with open(os.path.join(ROOT, "autos", auto_name + ".auto")) as f:
        auto = json.load(f)
    names = {}
    new_paths = {}
    for p in dict.fromkeys(path_names(auto["command"], [])):
        with open(os.path.join(ROOT, "paths", p + ".path")) as f:
            src = json.load(f)
        names[p] = swap(p, a, b)
        new_paths[names[p]] = mirror_path(src, a, b)
    mirror_commands(auto["command"], names)
    if auto.get("folder") and a in auto["folder"]:
        auto["folder"] = auto["folder"].replace(a, b)
    new_auto = swap(auto_name, a, b)

    for n, d in new_paths.items():
        with open(os.path.join(ROOT, "paths", n + ".path"), "w") as f:
            json.dump(d, f, indent=2)
        print(f"path  {n}")
    sync_linked_elsewhere(new_paths)
    with open(os.path.join(ROOT, "autos", new_auto + ".auto"), "w") as f:
        json.dump(auto, f, indent=2)
    print(f"auto  {new_auto}")
    check(auto, new_paths)


if __name__ == "__main__":
    main()
