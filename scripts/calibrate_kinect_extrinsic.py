#!/usr/bin/env python3
"""kinect1<->kinect2 extrinsic calibration via paired head_position_cam samples
(rigid point-set registration, Kabsch/SVD). Run while a person moves their head
around (side to side, forward/back, up/down) so points aren't collinear.

Usage:
    python3 calibrate_kinect_extrinsic.py [duration_sec]

Outputs the static_transform_publisher command to replace kinect2's guessed
identity-rotation TF, plus RMS residual error before/after.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


def kabsch(P, Q):
    """Find R,t minimizing sum |R@P_i + t - Q_i|^2. P,Q: (N,3)."""
    cP, cQ = P.mean(axis=0), Q.mean(axis=0)
    Pc, Qc = P - cP, Q - cQ
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = cQ - R @ cP
    return R, t


def rot_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


class Collector(Node):
    def __init__(self):
        super().__init__('kinect_extrinsic_calib')
        self.k1 = []
        self.k2 = []
        self.create_subscription(PointStamped, '/kinect1/head_position_cam',
                                  lambda m: self.k1.append(self._rec(m)), 10)
        self.create_subscription(PointStamped, '/kinect2/head_position_cam',
                                  lambda m: self.k2.append(self._rec(m)), 10)

    @staticmethod
    def _rec(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return (t, np.array([m.point.x, m.point.y, m.point.z]))


def match_pairs(k1, k2, tol=0.05):
    P1, P2 = [], []
    for t2, p2 in k2:
        best = min(k1, key=lambda kv: abs(kv[0] - t2), default=None)
        if best is not None and abs(best[0] - t2) <= tol:
            P1.append(best[1])
            P2.append(p2)
    return np.array(P1), np.array(P2)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    out_csv = sys.argv[2] if len(sys.argv) > 2 else "kinect_extrinsic_calib_pairs.csv"
    rclpy.init()
    node = Collector()
    print(f"[calib] {duration:.0f}초 동안 수집 중 — 지금부터 고개를 좌우/상하/앞뒤로 자연스럽게 움직여줘...")
    start = time.time()
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    print(f"[calib] kinect1 샘플 {len(node.k1)}개, kinect2 샘플 {len(node.k2)}개 수집")
    P1, P2 = match_pairs(node.k1, node.k2)
    print(f"[calib] 매칭된 쌍: {len(P1)}개")
    if len(P1) < 10:
        print("[calib][ERROR] 매칭된 샘플이 너무 적음 — 두 카메라 다 얼굴을 잘 보고 있었는지 확인 필요")
        return

    # raw correspondences 저장 (재현성 — 이전 실행은 이걸 안 남겨서 결과 재현 불가였음)
    with open(out_csv, "w") as f:
        f.write("k1_x,k1_y,k1_z,k2_x,k2_y,k2_z\n")
        for p1, p2 in zip(P1, P2):
            f.write(f"{p1[0]},{p1[1]},{p1[2]},{p2[0]},{p2[1]},{p2[2]}\n")
    print(f"[calib] raw 대응쌍 저장: {out_csv}")

    # before-calibration residual (identity rotation, guessed translation not applied here —
    # this is just local-frame vs local-frame raw offset for reference)
    raw_rms = np.sqrt(np.mean(np.sum((P1 - P2) ** 2, axis=1)))
    print(f"[calib] 보정 전 (단순 local-frame 차이) RMS: {raw_rms:.3f}m")

    # train/test split — in-sample fit residual만 보고하면 과대평가되므로 held-out으로도 확인
    rng = np.random.default_rng(0)
    idx = rng.permutation(len(P1))
    n_test = max(5, len(P1) // 5)
    test_idx, train_idx = idx[:n_test], idx[n_test:]
    R_tr, t_tr = kabsch(P2[train_idx], P1[train_idx])
    test_residual = P1[test_idx] - (P2[test_idx] @ R_tr.T + t_tr)
    test_rms = np.sqrt(np.mean(np.sum(test_residual ** 2, axis=1)))
    print(f"[calib] held-out 검증 RMS (train {len(train_idx)}개/test {len(test_idx)}개 분할): {test_rms:.4f}m")

    R, t = kabsch(P2, P1)  # 최종 변환은 전체 데이터로 (더 안정적) — 아래 rms는 in-sample fit residual, 참고용
    residual = P1 - (P2 @ R.T + t)
    rms = np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))
    print(f"[calib] (참고용, in-sample fit residual — 위 held-out 수치가 더 정직한 지표) ", end="")
    print(f"[calib] 보정 후 RMS 잔차: {rms:.4f}m")

    qx, qy, qz, qw = rot_to_quat(R)
    x, y, z = t
    print("\n[calib] static_transform_publisher 명령어 (map -> kinect2_rgb_optical_frame):")
    print(f"ros2 run tf2_ros static_transform_publisher "
          f"--frame-id map --child-frame-id kinect2_rgb_optical_frame "
          f"--x {x:.4f} --y {y:.4f} --z {z:.4f} "
          f"--qx {qx:.4f} --qy {qy:.4f} --qz {qz:.4f} --qw {qw:.4f}")


if __name__ == '__main__':
    main()
