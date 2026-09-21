"""gaze_fusion_node의 신뢰도 가중 융합 검증 (ROS2 필요, 카메라 불필요)."""
import importlib.util, os, sys, time
import numpy as np

HERE = os.path.expanduser('~/VPT-Framework/src/ros2_ws/src/vpt_gaze_bridge')
spec = importlib.util.spec_from_file_location('gfn', os.path.join(HERE, 'gaze_fusion_node.py'))
gfn = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gfn)

import rclpy
rclpy.init()
node = gfn.GazeFusionNode()

captured = {}
node.head_pub.publish = lambda m: captured.__setitem__('head', np.array([m.point.x, m.point.y, m.point.z]))
node.gaze_direction_pub.publish = lambda m: captured.__setitem__('gaze', np.array([m.vector.x, m.vector.y, m.vector.z]))
for p in (node.gaze_origin_pub, node.marker_pub, node.virtual_cam_info_pub):
    p.publish = lambda m: None
node.tf_broadcaster.sendTransform = lambda t: None

C1, C2 = gfn.CAMERAS

def setup(head1, gaze1, head2, gaze2, front1=None, front2=None):
    captured.clear()
    now = time.monotonic()
    for cam, h, g, f in ((C1, head1, gaze1, front1), (C2, head2, gaze2, front2)):
        e = node.cache[cam]
        if h is None:
            e.update(head=None, gaze=None, front=None, t_head=0.0, t_gaze=0.0, t_front=0.0)
            continue
        g = np.asarray(g, float); g = g / np.linalg.norm(g)
        e.update(head=np.asarray(h, float), gaze=g, t_head=now, t_gaze=now)
        if f is None:
            e.update(front=None, t_front=0.0)
        else:
            e.update(front=float(f), t_front=now)

A = np.array([0.0, 0.0, 0.0])          # C1 머리 위치
B = np.array([0.2, 0.0, 0.0])          # C2 머리 위치 (0.2m < HEAD_DISAGREE_M=0.3)
GA = np.array([1.0, 0.0, 0.0])                                   # C1 시선
GB = np.array([np.cos(np.radians(20)), np.sin(np.radians(20)), 0.0])  # C2 시선 (20° < 30°)

def angle_from_GA(v):
    return np.degrees(np.arccos(np.clip(np.dot(v, GA), -1, 1)))

fails = []
def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (('  | ' + detail) if detail else ''))
    if not cond:
        fails.append(name)

print('\n[1] frontality 없음 -> 기존 균등 평균 (하위호환)')
setup(A, GA, B, GB)
node.fuse_and_publish()
check('head = 정확히 중점', np.allclose(captured['head'], (A + B) / 2), f"head={captured['head']}")
check('gaze = 두 시선의 이등분(10°)', abs(angle_from_GA(captured['gaze']) - 10.0) < 1e-6,
      f"{angle_from_GA(captured['gaze']):.3f}°")

print('\n[2] frontality 가중 -> 정면인 카메라 쪽으로 쏠림')
setup(A, GA, B, GB, front1=20.0, front2=80.0)
node.fuse_and_publish()
_w1 = np.cos(np.radians(20.0)); _w2 = np.cos(np.radians(80.0))
w1, w2 = _w1 / (_w1 + _w2), _w2 / (_w1 + _w2)
check('head = 가중평균', np.allclose(captured['head'], w1 * A + w2 * B),
      f"head={captured['head']} 기대={w1 * A + w2 * B}")
check('정면(20°) 카메라 가중치가 5배 이상', w1 / w2 > 5, f"w1={w1:.3f} w2={w2:.3f}")
ang = angle_from_GA(captured['gaze'])
check('gaze가 정면 카메라 쪽으로 쏠림(균등평균 10°보다 작음)', ang < 5.0,
      f"{ang:.2f}° (균등평균이면 10°)")
check('gaze 단위벡터', abs(np.linalg.norm(captured['gaze']) - 1) < 1e-9)

print('\n[3] 대칭(둘 다 같은 frontality) -> 균등 평균과 동일')
setup(A, GA, B, GB, front1=45.0, front2=45.0)
node.fuse_and_publish()
check('head = 중점', np.allclose(captured['head'], (A + B) / 2), f"head={captured['head']}")
check('gaze = 이등분(10°)', abs(angle_from_GA(captured['gaze']) - 10.0) < 1e-6,
      f"{angle_from_GA(captured['gaze']):.3f}°")

print('\n[4] 큰 불일치 -> 더 정면인 카메라만 사용 (최근 갱신 아님)')
setup([0, 0, 0], [1, 0, 0], [5, 0, 0], [0, 1, 0], front1=15.0, front2=85.0)
node.cache[C2]['t_head'] = time.monotonic() + 10.0   # C2를 '더 최근'으로 만들어 구 로직과 구분
node.fuse_and_publish()
check('head = 더 정면인 C1 값', np.allclose(captured['head'], [0, 0, 0]), f"head={captured['head']}")
check('gaze = C1 값', np.allclose(captured['gaze'], [1, 0, 0]), f"gaze={captured['gaze']}")

print('\n[5] 불일치 + frontality 없음 -> 구 동작(더 최근 갱신) 유지')
setup([0, 0, 0], [1, 0, 0], [5, 0, 0], [0, 1, 0])
node.cache[C2]['t_head'] = time.monotonic() + 10.0
node.fuse_and_publish()
check('head = 더 최근인 C2 값', np.allclose(captured['head'], [5, 0, 0]), f"head={captured['head']}")

print('\n[6] 한 대만 살아있음 -> 그대로 통과')
setup([3, 1, 0], [0, 0, 1], None, None, front1=70.0)
node.fuse_and_publish()
check('head = C1 값', np.allclose(captured['head'], [3, 1, 0]), f"head={captured['head']}")
check('gaze = C1 값', np.allclose(captured['gaze'], [0, 0, 1]), f"gaze={captured['gaze']}")

print('\n[7] 둘 다 준측면(cos<=0) -> MIN_WEIGHT 하한으로 균등 평균, 크래시 없음')
setup(A, GA, B, GB, front1=110.0, front2=95.0)
node.fuse_and_publish()
check('head = 중점(균등)', np.allclose(captured['head'], (A + B) / 2), f"head={captured['head']}")

print('\n[8] frontality가 stale하면 가중치 미적용')
setup(A, GA, B, GB, front1=10.0, front2=85.0)
node.cache[C1]['t_front'] = time.monotonic() - 5.0   # STALE_SEC=1.0 초과
node.fuse_and_publish()
check('head = 균등 평균으로 복귀', np.allclose(captured['head'], (A + B) / 2), f"head={captured['head']}")

node.destroy_node(); rclpy.shutdown()
print('\n' + ('실패: ' + ', '.join(fails) if fails else '전체 통과'))
sys.exit(1 if fails else 0)
