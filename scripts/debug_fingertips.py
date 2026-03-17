#!/usr/bin/env python3
"""
Debug tool: captures fingertip position snapshots on keypress.
Press ENTER to capture a snapshot. Press Ctrl+C to quit.
Shows the difference from the previous snapshot so you can see which finger moved
and whether the whole fingertip cloud changes under wrist-only rotation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import numpy as np
import threading

FINGER_NAMES = ['Thumb (0)', 'Index (1)', 'Middle(2)', 'Ring  (3)', 'Pinky (4)']
NON_THUMB_IDX = [1, 2, 3, 4]

class FingertipDebugger(Node):
    def __init__(self):
        super().__init__('fingertip_debugger')
        self.latest = None
        self.prev_snapshot = None
        self.prev_metrics = None
        self.snapshot_count = 0
        self.msg_count = 0
        self.sub = self.create_subscription(
            PoseArray, '/manus/fingertip_positions', self.cb, 10)
        self.get_logger().info('Listening on /manus/fingertip_positions...')

    def cb(self, msg):
        if len(msg.poses) >= 5:
            self.latest = np.array([
                [msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z]
                for i in range(5)
            ])
            self.msg_count += 1

    def _metrics(self, snap):
        radii = np.linalg.norm(snap, axis=1)
        return {
            'radii': radii,
            'avg_non_thumb_radius': float(np.mean(radii[NON_THUMB_IDX])),
            'max_non_thumb_radius': float(np.max(radii[NON_THUMB_IDX])),
            'thumb_radius': float(radii[0]),
            'index_pinky_span': float(np.linalg.norm(snap[1] - snap[4])),
        }

    def take_snapshot(self):
        if self.latest is None:
            print("  No data received yet!")
            return
        snap = self.latest.copy()
        metrics = self._metrics(snap)
        self.snapshot_count += 1
        print(f"\n{'='*72}")
        print(f"  Snapshot #{self.snapshot_count}  (msgs received: {self.msg_count})")
        print(f"{'='*72}")
        print(f"  {'Finger':<12} {'X':>9} {'Y':>9} {'Z':>9}", end='')
        if self.prev_snapshot is not None:
            print(f"   {'dX':>8} {'dY':>8} {'dZ':>8} {'|delta|':>8} {'radius':>8}")
        else:
            print(f"   {'radius':>8}")

        for i in range(5):
            line = f"  {FINGER_NAMES[i]:<12} {snap[i,0]:>9.4f} {snap[i,1]:>9.4f} {snap[i,2]:>9.4f}"
            if self.prev_snapshot is not None:
                d = snap[i] - self.prev_snapshot[i]
                mag = np.linalg.norm(d)
                marker = ' <<<' if mag > 0.01 else ''
                line += (
                    f"   {d[0]:>8.4f} {d[1]:>8.4f} {d[2]:>8.4f} "
                    f"{mag:>8.4f}{marker} {metrics['radii'][i]:>8.4f}"
                )
            else:
                line += f"   {metrics['radii'][i]:>8.4f}"
            print(line)

        print("\n  Palm-radius summary:")
        print(
            "    avg non-thumb radius: "
            f"{metrics['avg_non_thumb_radius']:.4f} m"
        )
        print(
            "    max non-thumb radius: "
            f"{metrics['max_non_thumb_radius']:.4f} m"
        )
        print(
            "    thumb radius        : "
            f"{metrics['thumb_radius']:.4f} m"
        )
        print(
            "    index-pinky span    : "
            f"{metrics['index_pinky_span']:.4f} m"
        )

        if self.prev_snapshot is not None:
            diffs = np.linalg.norm(snap - self.prev_snapshot, axis=1)
            biggest = int(np.argmax(diffs))
            print(f"\n  >> Biggest change: {FINGER_NAMES[biggest]} ({diffs[biggest]:.4f}m)")
            print(
                "  >> Whole-cloud delta: "
                f"mean={float(np.mean(diffs)):.4f}m max={float(np.max(diffs)):.4f}m"
            )
            print(
                "  >> Radius drift: "
                f"avg_non_thumb={metrics['avg_non_thumb_radius'] - self.prev_metrics['avg_non_thumb_radius']:+.4f}m "
                f"span={metrics['index_pinky_span'] - self.prev_metrics['index_pinky_span']:+.4f}m"
            )
            if float(np.mean(diffs)) > 0.01:
                print("  >> Note: large whole-cloud motion can indicate wrist-frame contamination.")

        self.prev_snapshot = snap
        self.prev_metrics = metrics

def main():
    rclpy.init()
    node = FingertipDebugger()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\nFingertip Position Debugger")
    print("==========================")
    print("1. Keep hand flat and still, press ENTER for baseline snapshot")
    print("2. Make a relaxed fist, press ENTER again")
    print("3. Re-open, move only the index finger, press ENTER")
    print("4. Keep fingers fixed and rotate only the wrist/tracker, press ENTER")
    print("5. Watch the non-thumb radius and whole-cloud delta:")
    print("   - finger-only motion should mainly move one fingertip")
    print("   - wrist-only motion should not move the whole cloud much")
    print("   - open-hand non-thumb radius should be much larger than fist\n")
    print("Press ENTER to capture, Ctrl+C to quit.\n")

    try:
        while True:
            input(">> Press ENTER to capture snapshot...")
            node.take_snapshot()
    except (KeyboardInterrupt, EOFError):
        print("\nDone.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
