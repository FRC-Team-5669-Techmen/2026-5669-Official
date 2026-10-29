"""
Copy a PathPlanner auto and every path in it under a new name, to make a variation you can edit
without touching the original.

Usage (from the project folder; PathPlanner closed or it may overwrite the files):
    python tools/pathplanner/clone_auto.py "Right 3-Piece Auto" Agro

Makes "Agro Right 3-Piece Auto" with its own copies of every path ("Agro-Right-to-Pickup", ...).
Linked waypoints get their own names too ("Agro-<old name>"), so dragging a point in the copy
never moves the original. The copied paths go in a PathPlanner folder named after the prefix.
Refuses to overwrite files that already exist.
"""
import json
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                    "src", "main", "deploy", "pathplanner")


def path_names(cmd, out):
    t, data = cmd["type"], cmd.get("data", {})
    if t == "path":
        out.append(data["pathName"])
    elif t in ("sequential", "parallel", "deadline", "race"):
        for c in data["commands"]:
            path_names(c, out)
    return out


def rename_paths(cmd, names):
    t, data = cmd["type"], cmd.get("data", {})
    if t == "path":
        data["pathName"] = names[data["pathName"]]
    elif t in ("sequential", "parallel", "deadline", "race"):
        for c in data["commands"]:
            rename_paths(c, names)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    auto_name, prefix = sys.argv[1:]
    with open(os.path.join(ROOT, "autos", auto_name + ".auto")) as f:
        auto = json.load(f)
    names = {p: f"{prefix}-{p}" for p in dict.fromkeys(path_names(auto["command"], []))}
    new_auto = f"{prefix} {auto_name}"

    targets = [os.path.join(ROOT, "paths", n + ".path") for n in names.values()]
    targets.append(os.path.join(ROOT, "autos", new_auto + ".auto"))
    taken = [t for t in targets if os.path.exists(t)]
    if taken:
        print("Already exist, nothing written:\n  " + "\n  ".join(taken))
        sys.exit(1)

    for old, new in names.items():
        with open(os.path.join(ROOT, "paths", old + ".path")) as f:
            d = json.load(f)
        for w in d["waypoints"]:
            if w.get("linkedName"):
                w["linkedName"] = f"{prefix}-{w['linkedName']}"
        d["folder"] = prefix
        with open(os.path.join(ROOT, "paths", new + ".path"), "w") as f:
            json.dump(d, f, indent=2)
        print(f"path  {new}")
    rename_paths(auto["command"], names)
    with open(os.path.join(ROOT, "autos", new_auto + ".auto"), "w") as f:
        json.dump(auto, f, indent=2)
    print(f"auto  {new_auto}")

    # Show the copies in their own PathPlanner folder.
    settings_file = os.path.join(ROOT, "settings.json")
    with open(settings_file) as f:
        settings = json.load(f)
    if prefix not in settings["pathFolders"]:
        settings["pathFolders"].append(prefix)
        with open(settings_file, "w") as f:
            json.dump(settings, f, indent=2)
        print(f"PathPlanner folder \"{prefix}\" added")


if __name__ == "__main__":
    main()
