from pathlib import Path
import re


PROTOCOL_PATH = (
    Path(__file__).resolve().parents[2] / "perception_navigation_interface.md"
)


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
