"""Choose the channels declared on the *single* aiworker Robot State source.

The typed tactile source is also mirrored as numeric pressure observations
for the joint/pressure Parquet export. No ROS, SDK, network or hardware here.
"""

# CHANGED(2026-10-28): Single-source, schema-level training channel selection.
PRESETS = ("training", "qpos", "all")


def select_robot_channels(channel_infos, robot_type, selection="training"):
    """Return declared full channel keys in mapper order, or fail before connect.

    training: joint positions, numeric pressure/status and validity masks.
    qpos: only measured joint positions (including all configured hands).
    all: every robot channel, including optional diagnostics.
    custom: comma-separated exact keys, with or without '<robot_type>.' prefix.
    """
    if not isinstance(selection, str) or not selection.strip():
        raise ValueError("Loop channel selection must not be empty")
    selection = selection.strip()
    prefix = robot_type + "."
    declared = {}
    for info in channel_infos:
        full = info.key
        if not full.startswith(prefix):
            raise ValueError(f"unexpected robot channel outside {robot_type}: {full}")
        relative = full[len(prefix):]
        if relative in declared:
            raise ValueError(f"duplicate Loop robot channel: {full}")
        declared[relative] = full

    if selection == "training":
        chosen = {key for key in declared if (
            key.endswith(".joint_position") and
            (key.startswith("observation.") or key.startswith("action."))
            or key.endswith(('.tactile_pressure', '.tactile_status'))
            or key in ("teleop.flags", "teleop.action_valid", "teleop.observation_valid")
        )}
    elif selection == "qpos":
        chosen = {key for key in declared
                  if key.startswith("observation.") and key.endswith(".joint_position")}
    elif selection == "all":
        chosen = set(declared)
    else:
        chosen = set()
        for raw in selection.split(","):
            item = raw.strip()
            if not item:
                raise ValueError("empty Loop channel key in custom selection")
            relative = item[len(prefix):] if item.startswith(prefix) else item
            if relative not in declared:
                raise ValueError(
                    f"unknown Loop channel {item!r}; valid keys: {', '.join(declared)}")
            if relative in chosen:
                raise ValueError(f"duplicate Loop channel in selection: {item}")
            chosen.add(relative)
    if not chosen:
        raise ValueError("Loop channel selection has no declared channels")
    return tuple(full for relative, full in declared.items() if relative in chosen)
