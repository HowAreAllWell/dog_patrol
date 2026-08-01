from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[4]
PROTOCOL_PATH = REPO_ROOT / "docs" / "contracts" / "perception_navigation_interface.md"


def _fresh_bbox_rows():
    target_bbox_section = PROTOCOL_PATH.read_text(encoding="utf-8").split(
        "### 6.4 TargetNavigationStatus.msg", 1
    )[0]
    return {
        state: (accepted, purpose)
        for state, accepted, purpose in re.findall(
            r"\| `(STARTUP|PATROL|CONFIRM_TARGET|APPROACH_TARGET|VERIFY_IDENTITY|TRACK_INTRUDER)` "
            r"\| (是|否) \| ([^|]+) \|",
            target_bbox_section,
        )
    }


def test_verify_identity_bbox_supports_authorization_while_navigation_holds():
    rows = _fresh_bbox_rows()
    accepted_states = {
        state for state, (accepted, _) in rows.items() if accepted == "是"
    }

    assert accepted_states == {
        "CONFIRM_TARGET",
        "APPROACH_TARGET",
        "VERIFY_IDENTITY",
        "TRACK_INTRUDER",
    }

    accepted, purpose = rows["VERIFY_IDENTITY"]
    assert accepted == "是"
    assert "授权" in purpose
    assert "停车" in purpose
    assert "不因 bbox" in purpose


def test_target_lost_is_owned_by_perception():
    protocol = PROTOCOL_PATH.read_text(encoding="utf-8")
    target_lost_section = protocol.split("### 8.7 TARGET_LOST", 1)[1].split(
        "### 8.8 EXECUTION_ERROR", 1
    )[0]

    assert "只能由感知发布" in target_lost_section
    assert "导航不得发布 `TARGET_LOST`" in target_lost_section
