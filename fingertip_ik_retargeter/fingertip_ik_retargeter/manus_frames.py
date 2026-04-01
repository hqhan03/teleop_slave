"""Shared Manus hand-frame definitions for fingertip IK retargeting."""

CANONICAL_MANUS_HAND_FRAME = 'manus_wrist_local'
LEGACY_MANUS_HAND_FRAME = 'manus_palm'
ACCEPTED_MANUS_HAND_FRAMES = ('', CANONICAL_MANUS_HAND_FRAME, LEGACY_MANUS_HAND_FRAME)


def classify_manus_hand_frame(frame_id: str | None) -> tuple[bool, bool]:
    """
    Classify an incoming Manus hand-local frame id.

    Returns:
        (accepted, is_legacy_alias)
    """
    normalized = frame_id or ''
    if normalized in ('', CANONICAL_MANUS_HAND_FRAME):
        return True, False
    if normalized == LEGACY_MANUS_HAND_FRAME:
        return True, True
    return False, False
