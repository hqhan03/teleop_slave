import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

from keyvector_retargeter.manus_landmarks import CANONICAL_MANUS_HAND_FRAME, decode_manus_landmarks_msg


def _pose(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def test_decode_pose_array_order_and_mapping():
    msg = PoseArray()
    msg.header = Header(frame_id=CANONICAL_MANUS_HAND_FRAME)
    for idx in range(25):
        msg.poses.append(_pose(float(idx), float(idx + 1), float(idx + 2)))

    hand = decode_manus_landmarks_msg(msg)
    loss_keypoints = hand.loss_keypoints()

    np.testing.assert_allclose(loss_keypoints['thumb']['mcp'], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(loss_keypoints['thumb']['pip'], [2.0, 3.0, 4.0])
    np.testing.assert_allclose(loss_keypoints['thumb']['dip'], [3.0, 4.0, 5.0])
    np.testing.assert_allclose(loss_keypoints['thumb']['tip'], [4.0, 5.0, 6.0])
    np.testing.assert_allclose(loss_keypoints['index']['mcp'], [6.0, 7.0, 8.0])


def test_all_zero_detection():
    msg = PoseArray()
    msg.header = Header(frame_id=CANONICAL_MANUS_HAND_FRAME)
    for _ in range(25):
        msg.poses.append(_pose(0.0, 0.0, 0.0))
    hand = decode_manus_landmarks_msg(msg)
    assert hand.is_all_zero()
