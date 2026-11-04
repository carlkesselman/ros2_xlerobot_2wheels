"""Plans (schema version 1): validation, no ROS, so it can be tested on its own.

A plan is a JSON object:

    {
      "schema_version": 1,
      "task": "touch the yellow cube",
      "objects": {"cube": {"prompt": "a cube", "pose": [0.24, -0.05, 0.77], "frame": "map", "size_m": 0.028}},
      "steps": [
        {"id": "look", "skill": "look_at", "params": {"named_position": "head_look_down"}},
        {"id": "find", "skill": "detect", "params": {"prompt": "a cube", "bind": "cube"},
         "on_failure": {"codes": ["NOT_FOUND"], "action": "retry", "max_retries": 2}},
        {"id": "reach", "skill": "approach", "params": {"arm": "right", "object": "cube"},
         "expect_text": "the claw tip is above the cube", "on_pause": "retreat:right_rest"},
        {"id": "done", "skill": "move_arm", "params": {"arm": "right", "named_position": "right_rest"}}
      ]
    }

Steps run in order. A step succeeds when its skill returns SUCCEEDED and its `expect` checks pass. A failure whose
result code is in `on_failure.codes` (every code when the list is empty) is handled by `on_failure.action`: `retry`
(up to `max_retries` times), `goto` another step (counted against `max_retries` too), or `abort` (the default). Any
other failure stops the plan and is reported to the planner. SAFETY_STOP and PREEMPTED can never be handled by a plan.
"""
import math

SCHEMA_VERSION = 1
MAX_RETRIES = 5
CODES = ["SUCCEEDED", "TIMEOUT", "NOT_REACHABLE", "NOT_FOUND", "AMBIGUOUS", "LOST_OBJECT", "COLLISION", "SAFETY_STOP",
         "PREEMPTED", "FAILED"]
UNHANDLED = {"SAFETY_STOP", "PREEMPTED"}
ARMS = ("left", "right")

# skill -> parameter -> (type, required)
SKILLS = {
    "navigate_to": {"waypoint": (str, False), "pose": (list, False), "frame": (str, False), "timeout_s": (float, False),
                    "final_straight_m": (float, False)},
    "look_at": {"named_position": (str, False), "target": (list, False), "frame": (str, False), "object": (str, False),
                "tolerance_deg": (float, False)},
    "detect": {"prompt": (str, True), "label": (str, False), "size_m": (float, False), "bind": (str, False)},
    "move_arm": {"arm": (str, True), "named_position": (str, False), "point": (list, False), "frame": (str, False),
                 "tool_point": (bool, False), "table_z": (float, False), "scaling": (float, False), "timeout_s": (float, False)},
    "approach": {"arm": (str, True), "object": (str, True), "height_m": (float, False), "scaling": (float, False)},
    "touch": {"arm": (str, True), "object": (str, True), "max_travel_m": (float, False), "step_m": (float, False),
              "stay": (bool, False)},
    "grasp": {"arm": (str, True), "object": (str, True), "lift_m": (float, False), "scaling": (float, False)},
    "place": {"arm": (str, True), "location": (str, False), "object": (str, False), "point": (list, False),
              "frame": (str, False), "held_size_m": (float, False), "scaling": (float, False)},
}
PREDICATES = ("detected", "joint_near")


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _typed(v, t):
    if t is float:
        return _num(v)
    if t is bool:
        return isinstance(v, bool)
    return isinstance(v, t)


def _check_predicates(where, items, objects, errors):
    if not isinstance(items, list):
        errors.append(f"{where}: must be a list of checks")
        return
    for c in items:
        if not isinstance(c, dict) or len(c) != 1 or next(iter(c)) not in PREDICATES:
            errors.append(f"{where}: {c!r} is not one of {', '.join(PREDICATES)}")
            continue
        (name, arg), = c.items()
        if name == "detected" and arg not in objects:
            errors.append(f"{where}: detected({arg!r}) names no object")
        if name == "joint_near" and not (isinstance(arg, dict) and isinstance(arg.get("position"), str)
                                         and _num(arg.get("tol", 0.05))):
            errors.append(f"{where}: joint_near needs {{\"position\": name, \"tol\": rad}}")


def validate(plan, known_positions=None, known_waypoints=None):
    """Errors as a list of strings (empty = valid). known_* (sets of names), when given, are checked too."""
    errors = []
    if not isinstance(plan, dict):
        return ["the plan must be a JSON object"]
    if plan.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")
    objects = plan.get("objects", {})
    if not isinstance(objects, dict):
        errors.append("objects must be an object of name -> {prompt, pose, frame, size_m}")
        objects = {}
    for name, o in objects.items():
        if not isinstance(o, dict) or not isinstance(o.get("prompt"), str):
            errors.append(f"object {name!r}: needs a prompt")
            continue
        if "pose" in o and not (isinstance(o["pose"], list) and len(o["pose"]) == 3 and all(_num(v) for v in o["pose"])):
            errors.append(f"object {name!r}: pose must be [x, y, z]")
    steps = plan.get("steps")
    if not isinstance(steps, list) or not steps:
        return errors + ["steps must be a non-empty list"]
    ids = [s.get("id") if isinstance(s, dict) else None for s in steps]
    if len(set(ids)) != len(ids) or not all(isinstance(i, str) and i for i in ids):
        errors.append("every step needs a unique, non-empty id")
    for k, s in enumerate(steps):
        where = f"step {s.get('id', k) if isinstance(s, dict) else k}"
        if not isinstance(s, dict):
            errors.append(f"{where}: must be an object")
            continue
        skill = s.get("skill")
        if skill not in SKILLS:
            errors.append(f"{where}: unknown skill {skill!r} (known: {', '.join(SKILLS)})")
            continue
        params = s.get("params", {})
        if not isinstance(params, dict):
            errors.append(f"{where}: params must be an object")
            continue
        spec = SKILLS[skill]
        for p, v in params.items():
            if p not in spec:
                errors.append(f"{where}: {skill} has no parameter {p!r}")
            elif not _typed(v, spec[p][0]):
                errors.append(f"{where}: {p} must be a {spec[p][0].__name__}")
        for p, (_, required) in spec.items():
            if required and p not in params:
                errors.append(f"{where}: {skill} needs {p!r}")
        if "arm" in params and params["arm"] not in ARMS:
            errors.append(f"{where}: arm must be left or right")
        for p in ("object", "bind"):
            if p in params and params[p] not in objects:
                errors.append(f"{where}: {p} {params[p]!r} names no object in the plan")
        if skill == "place" and sum(p in params for p in ("location", "object", "point")) != 1:
            errors.append(f"{where}: place needs exactly one of location, object or point")
        if skill in ("approach", "touch", "look_at", "grasp", "place") and "object" in params and "pose" not in objects.get(params["object"], {}) \
                and not any(isinstance(t, dict) and t.get("skill") == "detect" and t.get("params", {}).get("bind") == params["object"]
                            for t in steps[:k]):
            errors.append(f"{where}: object {params['object']!r} has no pose and no earlier detect binds it")
        if skill == "move_arm" and ("named_position" in params) == ("point" in params):
            errors.append(f"{where}: move_arm needs exactly one of named_position or point")
        if skill == "look_at" and sum(p in params for p in ("named_position", "target", "object")) != 1:
            errors.append(f"{where}: look_at needs exactly one of named_position, target or object")
        if skill == "navigate_to" and ("waypoint" in params) == ("pose" in params):
            errors.append(f"{where}: navigate_to needs exactly one of waypoint or pose")
        for p in ("point", "target"):
            if p in params and not (len(params[p]) == 3 and all(_num(v) for v in params[p])):
                errors.append(f"{where}: {p} must be [x, y, z]")
        if "pose" in params and not (len(params["pose"]) == 3 and all(_num(v) for v in params["pose"])):
            errors.append(f"{where}: pose must be [x, y, yaw]")
        if known_positions is not None and "named_position" in params and params["named_position"] not in known_positions:
            errors.append(f"{where}: no saved position {params['named_position']!r}")
        if known_waypoints is not None and "waypoint" in params and params["waypoint"] not in known_waypoints:
            errors.append(f"{where}: no saved waypoint {params['waypoint']!r}")
        if "timeout_s" in s and not (_num(s["timeout_s"]) and s["timeout_s"] > 0):
            errors.append(f"{where}: timeout_s must be a positive number")
        for key in ("pre", "expect"):
            if key in s:
                _check_predicates(f"{where} {key}", s[key], objects, errors)
        f = s.get("on_failure", {"action": "abort"})
        if not isinstance(f, dict):
            errors.append(f"{where}: on_failure must be an object")
            continue
        codes = f.get("codes", [])
        bad = [c for c in codes if c not in CODES or c == "SUCCEEDED"]
        if bad:
            errors.append(f"{where}: on_failure codes {bad} are not failure codes")
        if set(codes) & UNHANDLED:
            errors.append(f"{where}: {sorted(set(codes) & UNHANDLED)} cannot be handled by a plan")
        action = f.get("action", "abort")
        if action not in ("retry", "goto", "abort"):
            errors.append(f"{where}: on_failure action must be retry, goto or abort")
        if action in ("retry", "goto"):
            n = f.get("max_retries", 1)
            if not (isinstance(n, int) and not isinstance(n, bool) and 1 <= n <= MAX_RETRIES):
                errors.append(f"{where}: max_retries must be 1 to {MAX_RETRIES}")
        if action == "goto" and f.get("step") not in ids:
            errors.append(f"{where}: on_failure goto names no step")
        pause = s.get("on_pause", "hold")
        if not (pause in ("hold", "stop_base") or (isinstance(pause, str) and pause.startswith("retreat:") and len(pause) > 8)):
            errors.append(f"{where}: on_pause must be hold, stop_base or retreat:<saved position>")
        elif known_positions is not None and isinstance(pause, str) and pause.startswith("retreat:") \
                and pause[8:] not in known_positions:
            errors.append(f"{where}: no saved position {pause[8:]!r} to retreat to")
    # one motion per servo bus at a time is the skill server's rule; steps here run one after another
    return errors


def handles(step, code):
    """(action, failure spec) for a failed step: abort unless on_failure covers the code."""
    f = step.get("on_failure", {})
    codes = f.get("codes", [])
    if code in UNHANDLED or (codes and code not in codes):
        return "abort", f
    return f.get("action", "abort"), f
