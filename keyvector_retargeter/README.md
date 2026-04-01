# keyvector_retargeter

`keyvector_retargeter` converts MANUS 25-landmark hand poses into Tesollo DG-5F joint trajectories by minimizing a keyvector loss in the DG-5F palm frame.

The package is independent from the fingertip IK path. It uses:

- `/manus/hand_landmarks`
- the right-hand DG-5F URDF
- a SciPy bounded least-squares solver

Main executables:

- `keyvector_retarget_node`
- `calibrate_keyvector`

Calibration flow:

- `stage1` commands the robot to `reference_joint_positions_deg`, waits for you to mirror that pose with the glove, and captures after you press Enter.
- For the most stable stage-1 frame fit, keep that reference pose open or clearly spread rather than tightly pinched.
- `stage2` walks through the built-in multipose sequence, including URDF-validated thumb-index pinch, thumb-middle pinch, and tripod precision grasp poses.
