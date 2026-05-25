"""pytest checks on the OpenVR action manifest + binding files.

These tests don't run SteamVR — they just verify the JSON config files
ship with the right structure so a SteamVR + PICO controller setup will
auto-load our binding when the user opens Manage Controller Bindings.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
CONFIG_DIR = ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions"
ACTIONS_PATH = CONFIG_DIR / "actions.json"
BINDINGS_PATH = CONFIG_DIR / "bindings_pico.json"
MANIFEST_PATH = CONFIG_DIR / "manifest.vrmanifest"


@pytest.fixture(scope="module")
def actions_json():
    with open(ACTIONS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="module")
def bindings_json():
    with open(BINDINGS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


@pytest.fixture(scope="module")
def manifest_json():
    with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


# ── actions.json ───────────────────────────────────────────────────────
def test_actions_json_app_key(actions_json):
    assert actions_json["app_key"] == "ust.teleop.gr1t2_gripper"


def test_actions_json_has_pose_actions(actions_json):
    names = {a["name"] for a in actions_json["actions"]}
    assert "/actions/teleop/in/pose_left" in names
    assert "/actions/teleop/in/pose_right" in names


def test_actions_json_has_trigger_grip_menu(actions_json):
    names = {a["name"] for a in actions_json["actions"]}
    for n in (
        "/actions/teleop/in/trigger_left",
        "/actions/teleop/in/trigger_right",
        "/actions/teleop/in/grip_left",
        "/actions/teleop/in/grip_right",
        "/actions/teleop/in/menu_left",
        "/actions/teleop/in/menu_right",
    ):
        assert n in names, f"missing action {n}"


def test_actions_json_has_no_finger_curl_actions(actions_json):
    """Migration removed the 10 per-finger curl actions used by UDCAP."""
    names = {a["name"] for a in actions_json["actions"]}
    for forbidden in (
        "/actions/teleop/in/finger_thumb_left",
        "/actions/teleop/in/finger_index_left",
        "/actions/teleop/in/finger_thumb_right",
        "/actions/teleop/in/finger_pinky_right",
    ):
        assert forbidden not in names


def test_actions_json_has_no_skeleton_actions(actions_json):
    names = {a["name"] for a in actions_json["actions"]}
    for forbidden in (
        "/actions/teleop/in/skeleton_left",
        "/actions/teleop/in/skeleton_right",
    ):
        assert forbidden not in names


def test_actions_json_default_bindings_include_pico(actions_json):
    """All known PICO / Touch / Knuckles controller_type variants must be
    listed (memory.md §10.42).

    Background: SteamVR only loads our binding when the live controller's
    Prop_ControllerType_String matches one of these entries.  Missing
    'pico_controller' (Pico Connect 6.x+ on PICO 4 Ultra) caused 9.33's
    prism-path switch to silently produce all-zero action values.
    """
    types = {b["controller_type"] for b in actions_json["default_bindings"]}
    # Primary modern entries observed in the wild
    assert "pico_controller" in types, (
        "Missing 'pico_controller' — required for Pico Connect 6.x+ / PICO 4 "
        "Ultra (memory.md §10.42)."
    )
    assert "pico_neo3_controller" in types  # older Pico Connect 5.x
    assert "oculus_touch" in types          # VD emulation / Compat Mode
    # Futureproofing entries (don't fail tests if removed later)
    # but document expected presence
    expected_extra = {"pico_phoenix_controller", "pico4_controller", "knuckles"}
    missing_extra = expected_extra - types
    assert not missing_extra, (
        f"Missing futureproofing controller_type entries: {missing_extra}.  "
        f"These are cheap to keep — add them back unless explicitly dropped."
    )


def test_actions_json_default_bindings_use_correct_url(actions_json):
    """9.42: each controller_type maps to its own bindings_<controller_type>.json
    so SteamVR's file-level controller_type validation succeeds.  See
    memory.md section 10.49 for the silent-reject root cause analysis."""
    expected_map = {
        "pico_controller":         "bindings_pico_controller.json",
        "pico_phoenix_controller": "bindings_pico_phoenix.json",
        "pico4_controller":        "bindings_pico4.json",
        "pico_neo3_controller":    "bindings_pico_neo3.json",
        "oculus_touch":            "bindings_oculus_touch.json",
        "knuckles":                "bindings_knuckles.json",
    }
    actual_map = {b["controller_type"]: b["binding_url"]
                  for b in actions_json["default_bindings"]}
    for ctype, expected_url in expected_map.items():
        assert actual_map.get(ctype) == expected_url, (
            f"controller_type={ctype!r} maps to {actual_map.get(ctype)!r}; "
            f"expected {expected_url!r}.  9.42 split each controller_type into "
            f"its own binding file to fix SteamVR's silent-reject of "
            f"controller_type-mismatched binding files."
        )


def test_each_binding_file_has_matching_controller_type():
    """9.42: each binding file's top-level controller_type must match its
    filename and the default_bindings entry that points to it.  SteamVR
    silently rejects binding files whose top-level controller_type does
    not match the lookup key (memory.md section 10.49)."""
    cfg_dir = ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions"
    expected = {
        "bindings_pico_controller.json":  "pico_controller",
        "bindings_pico_phoenix.json":     "pico_phoenix_controller",
        "bindings_pico4.json":            "pico4_controller",
        "bindings_pico_neo3.json":        "pico_neo3_controller",
        "bindings_oculus_touch.json":     "oculus_touch",
        "bindings_knuckles.json":         "knuckles",
    }
    for fname, ctype in expected.items():
        f = cfg_dir / fname
        assert f.exists(), (
            f"{fname} missing -- 9.42 created six per-controller_type "
            f"binding files; SteamVR cannot route {ctype!r} without it."
        )
        data = json.loads(f.read_text(encoding="utf-8"))
        assert data.get("controller_type") == ctype, (
            f"{fname}: top-level controller_type={data.get('controller_type')!r}, "
            f"expected {ctype!r}.  Mismatch causes SteamVR to silently reject "
            f"the binding (the bug fixed in 9.42)."
        )


def test_all_binding_variants_share_identical_action_paths():
    """9.42: only top-level metadata (controller_type / name / description)
    differs across the six binding variants; sources/poses must be identical
    so behaviour is consistent regardless of which controller_type SteamVR
    sees at runtime."""
    cfg_dir = ROOT / "ust_ws" / "ust_hm_grip" / "config" / "openvr_actions"
    files = [
        "bindings_pico_controller.json",
        "bindings_pico_phoenix.json",
        "bindings_pico4.json",
        "bindings_pico_neo3.json",
        "bindings_oculus_touch.json",
        "bindings_knuckles.json",
    ]
    sigs = []
    for fname in files:
        data = json.loads((cfg_dir / fname).read_text(encoding="utf-8"))
        sources = data["bindings"]["/actions/teleop"]["sources"]
        poses = data["bindings"]["/actions/teleop"]["poses"]
        sig = (
            tuple(sorted((s["path"], s["mode"]) for s in sources)),
            tuple(sorted((p["path"], p["output"]) for p in poses)),
        )
        sigs.append((fname, sig))
    first_fname, first_sig = sigs[0]
    for fname, sig in sigs[1:]:
        assert sig == first_sig, (
            f"{fname} sources/poses differ from {first_fname}.  All six "
            f"binding variants must share identical input mappings (only "
            f"controller_type / name / description top-level metadata may "
            f"differ).  See memory.md section 10.49."
        )


# ── bindings_pico.json ─────────────────────────────────────────────────
def test_bindings_has_trigger_pull_outputs(bindings_json):
    sources = bindings_json["bindings"]["/actions/teleop"]["sources"]
    paths = {s["path"]: s for s in sources}
    for hand in ("left", "right"):
        path = f"/user/hand/{hand}/input/trigger"
        assert path in paths, f"missing source for {path}"
        s = paths[path]
        assert s["mode"] == "trigger"
        assert s["inputs"]["pull"]["output"] == f"/actions/teleop/in/trigger_{hand}"


def test_bindings_has_grip_pull_outputs(bindings_json):
    """Grip is bound as trigger/pull, not force_sensor/force (post-9.32).

    See memory.md §10.40: force_sensor requires /input/grip/force, which
    only exists on Knuckles and pico_neo3_controller.  VD's Oculus-Touch
    emulation only exposes /input/grip/value, so the previous binding
    silently returned 0 under VD.  trigger/pull reads /input/grip/value,
    which is universal across all three default-bound controller_type
    values.
    """
    sources = bindings_json["bindings"]["/actions/teleop"]["sources"]
    paths = {s["path"]: s for s in sources}
    for hand in ("left", "right"):
        path = f"/user/hand/{hand}/input/grip"
        assert path in paths
        s = paths[path]
        assert s["mode"] == "trigger", (
            f"grip binding for {hand} must be mode='trigger' (analog pull); "
            f"got {s['mode']!r}.  See memory.md §10.40."
        )
        assert s["inputs"]["pull"]["output"] == f"/actions/teleop/in/grip_{hand}"


def test_bindings_has_application_menu_click(bindings_json):
    sources = bindings_json["bindings"]["/actions/teleop"]["sources"]
    paths = {s["path"]: s for s in sources}
    for hand in ("left", "right"):
        path = f"/user/hand/{hand}/input/application_menu"
        assert path in paths
        s = paths[path]
        assert s["mode"] == "button"
        assert s["inputs"]["click"]["output"] == f"/actions/teleop/in/menu_{hand}"


def test_bindings_has_pose_raw(bindings_json):
    poses = bindings_json["bindings"]["/actions/teleop"]["poses"]
    expected = {
        f"/actions/teleop/in/pose_{hand}": f"/user/hand/{hand}/pose/raw"
        for hand in ("left", "right")
    }
    for p in poses:
        assert p["output"] in expected
        assert p["path"] == expected[p["output"]]


# ── manifest.vrmanifest ────────────────────────────────────────────────
def test_manifest_app_key_matches_actions(manifest_json, actions_json):
    apps = manifest_json["applications"]
    assert len(apps) == 1
    assert apps[0]["app_key"] == actions_json["app_key"]


def test_manifest_action_manifest_path_relative(manifest_json):
    assert manifest_json["applications"][0]["action_manifest_path"] == "actions.json"


def test_manifest_is_not_dashboard_overlay(manifest_json):
    """is_dashboard_overlay = false ensures our binding takes priority
    over SteamVR's dashboard pointer when our app is in focus."""
    assert manifest_json["applications"][0]["is_dashboard_overlay"] is False


# ── tracker_binding.json ───────────────────────────────────────────────
TRACKER_BINDING_PATH = ROOT / "ust_ws" / "ust_hm_grip" / "config" / "tracker_binding.json"
TRACKER_BINDING_PICO_PATH = (
    ROOT / "ust_ws" / "ust_hm_grip" / "config" / "tracker_binding_pico_connect.json"
)


def test_tracker_binding_uses_elbow_role():
    with open(TRACKER_BINDING_PATH, "r", encoding="utf-8") as f:
        tb = json.load(f)
    trackers = tb["trackers"]
    assert trackers["left_arm_lower"]["role"] == "left_forearm"
    assert trackers["left_arm_lower"]["steamvr_role"] == "TrackerRole_LeftElbow"
    assert trackers["right_arm_lower"]["role"] == "right_forearm"
    assert trackers["right_arm_lower"]["steamvr_role"] == "TrackerRole_RightElbow"

# ---- tracker_binding_pico_connect.json (9.37) ----
def test_tracker_binding_pico_connect_template_exists():
    """9.37: the PICO Connect template ships alongside the legacy file."""
    assert TRACKER_BINDING_PICO_PATH.exists(), (
        f"{TRACKER_BINDING_PICO_PATH} missing -- run_teleop.py "
        "--vr_runtime pico_connect would silently fall back to the VD "
        "body-segment template.  Restore the file (see memory.md section 10.45)."
    )


def test_tracker_binding_pico_connect_has_required_grip_roles_or_template():
    """The grip retargeter consumes left_forearm + right_forearm as the
    wrist-EEF fallback.  The shipped template ships with PMT_REPLACE_ME
    placeholders that map those two roles; once the user runs
    enumerate_trackers --out, the file is overwritten in place with
    real PICO serials (Waist / LeftWrist / etc.) and a TODO_pico tag
    on each (the user then hand-edits the role to one of the grip-track
    valid values).

    9.39: relaxed from the original 9.37 strict "must contain
    left_forearm + right_forearm" check so the auto-populated user
    workflow does not break the test suite.  Three valid states:

      A) shipped template          -> contains 'left_forearm' role
      B) auto-populated, unedited  -> contains 'TODO_pico' role
      C) hand-edited final state   -> contains 'left_forearm' role
    """
    with open(TRACKER_BINDING_PICO_PATH, "r", encoding="utf-8") as f:
        tb = json.load(f)
    trackers = tb["trackers"]
    roles = {info.get("role", "") for info in trackers.values()}
    has_required = "left_forearm" in roles and "right_forearm" in roles
    has_todo_pico = any(r.lower().startswith("todo") for r in roles if r)
    assert has_required or has_todo_pico, (
        "tracker_binding_pico_connect.json has neither (left_forearm + "
        "right_forearm) NOR TODO_pico placeholders.  After running "
        "enumerate_trackers --out the file should at minimum carry "
        "TODO_pico tags so the user knows which entries to hand-assign."
    )



def test_tracker_binding_pico_connect_has_pmt_or_pico_serials():
    """Template ships with PMT_REPLACE_ME_* placeholders OR has been
    populated by enumerate_trackers --out with real PICO Motion Tracker
    serials.  9.39 relaxation: accept either state."""
    with open(TRACKER_BINDING_PICO_PATH, "r", encoding="utf-8") as f:
        tb = json.load(f)
    trackers = tb["trackers"]
    pmt_keys = [k for k in trackers if k.upper().startswith("PMT_REPLACE_ME")]
    # PICO Connect 4-Ultra in the field reports serials like
    # 'Waist' / 'LeftWrist' / 'LeftFoot' / 'RightFoot' / 'RightWrist'
    # (these are the role names PICO assigns to the trackers in its
    # full-body tracking mode).  Accept either as evidence the file is
    # in a sensible state.
    pico_keys = [
        k for k in trackers
        if k.upper().startswith(("WAIST", "LEFTFOOT", "RIGHTFOOT",
                                  "LEFTWRIST", "RIGHTWRIST",
                                  "PMT_", "PICOBT_"))
    ]
    assert len(pmt_keys) >= 2 or len(pico_keys) >= 2, (
        "tracker_binding_pico_connect.json should ship with at least 2 "
        "PMT_REPLACE_ME_* placeholder serials or have been populated with "
        "at least 2 PICO Motion Tracker serials by enumerate_trackers."
    )



def test_tracker_binding_pico_connect_legs_are_unused_or_todo():
    """Leg slots are unused by the grip retargeter.

    9.39 relaxation: allow either of two states.

      A) shipped template -- leg slots have role="" so SteamVRSampler
         skips them.
      B) auto-populated by enumerate_trackers -- leg slots carry the
         'TODO_pico' tag (user must hand-assign or clear).

    Reject only the explicit error: a leg slot bound to 'left_forearm'
    or 'right_forearm' or 'waist', which would mis-route lower-body
    pose data into the grip retargeter's wrist-EEF fallback channel.
    """
    with open(TRACKER_BINDING_PICO_PATH, "r", encoding="utf-8") as f:
        tb = json.load(f)
    trackers = tb["trackers"]
    forbidden_for_legs = {"left_forearm", "right_forearm", "waist"}
    for sn, info in trackers.items():
        steamvr_role = info.get("steamvr_role", "")
        role = info.get("role", "")
        is_leg_slot = (
            steamvr_role in ("TrackerRole_LeftFoot", "TrackerRole_RightFoot")
            or sn.upper() in ("LEFTFOOT", "RIGHTFOOT")
            or "ANKLE" in sn.upper()
            or "FOOT" in sn.upper()
        )
        if is_leg_slot:
            assert role not in forbidden_for_legs, (
                f"Leg slot {sn} is mis-bound to role={role!r}; the grip "
                f"retargeter does not consume lower-body trackers, so "
                f"this binding would have no effect.  Set role='' or a "
                f"TODO_pico tag and bind a different tracker to that "
                f"forearm/waist role."
            )

