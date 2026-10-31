import copy

from xlerobot_executive.plan import handles, validate

GOOD = {
    "schema_version": 1,
    "task": "touch the yellow cube",
    "objects": {"cube": {"prompt": "a cube", "pose": [0.24, -0.05, 0.77], "frame": "map", "size_m": 0.028}},
    "steps": [
        {"id": "look", "skill": "look_at", "params": {"named_position": "head_look_down"}},
        {"id": "find", "skill": "detect", "params": {"prompt": "a cube", "bind": "cube"},
         "on_failure": {"codes": ["NOT_FOUND"], "action": "retry", "max_retries": 2}},
        {"id": "reach", "skill": "approach", "params": {"arm": "right", "object": "cube"},
         "expect": [{"detected": "cube"}], "on_pause": "retreat:right_rest"},
        {"id": "back", "skill": "move_arm", "params": {"arm": "right", "named_position": "right_rest"},
         "expect": [{"joint_near": {"position": "right_rest", "tol": 0.05}}]},
    ],
}


def bad(change):
    p = copy.deepcopy(GOOD)
    change(p)
    return validate(p, known_positions={"head_look_down", "right_rest"})


def test_good_plan():
    assert validate(GOOD, known_positions={"head_look_down", "right_rest"}) == []


def test_rejections():
    assert any("schema_version" in e for e in bad(lambda p: p.update(schema_version=2)))
    assert any("unknown skill" in e for e in bad(lambda p: p["steps"][0].update(skill="dance")))
    assert any("no parameter" in e for e in bad(lambda p: p["steps"][0]["params"].update(speed=3)))
    assert any("needs 'arm'" in e for e in bad(lambda p: p["steps"][2]["params"].pop("arm")))
    assert any("left or right" in e for e in bad(lambda p: p["steps"][2]["params"].update(arm="middle")))
    assert any("names no object" in e for e in bad(lambda p: p["steps"][2]["params"].update(object="ball")))
    assert any("unique" in e for e in bad(lambda p: p["steps"][1].update(id="look")))
    assert any("cannot be handled" in e for e in bad(lambda p: p["steps"][1]["on_failure"].update(codes=["SAFETY_STOP"])))
    assert any("max_retries" in e for e in bad(lambda p: p["steps"][1]["on_failure"].update(max_retries=50)))
    assert any("goto names no step" in e for e in bad(lambda p: p["steps"][1].update(on_failure={"action": "goto", "step": "x"})))
    assert any("no saved position" in e for e in bad(lambda p: p["steps"][3]["params"].update(named_position="nowhere")))
    assert any("exactly one" in e for e in bad(lambda p: p["steps"][3]["params"].update(point=[0.2, 0, 0.8])))
    assert any("on_pause" in e for e in bad(lambda p: p["steps"][2].update(on_pause="fly")))
    assert any("joint_near" in e for e in bad(lambda p: p["steps"][3].update(expect=[{"joint_near": 3}])))


def test_object_needs_pose_or_earlier_detect():
    p = copy.deepcopy(GOOD)
    del p["objects"]["cube"]["pose"]
    assert validate(p) == []                    # the detect step binds it
    p["steps"].pop(1)
    assert any("no pose" in e for e in validate(p))


def test_handles():
    step = GOOD["steps"][1]
    assert handles(step, "NOT_FOUND")[0] == "retry"
    assert handles(step, "TIMEOUT")[0] == "abort"
    assert handles(step, "SAFETY_STOP")[0] == "abort"
    assert handles({"on_failure": {"action": "retry"}}, "TIMEOUT")[0] == "retry"
    assert handles({"on_failure": {"action": "retry"}}, "PREEMPTED")[0] == "abort"
