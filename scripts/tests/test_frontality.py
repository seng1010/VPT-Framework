"""gaze_bridge_node.compute_frontality_deg 검증 (모델 로드 없이 메서드만 스텁으로 호출)."""
import importlib.util, os, sys
from collections import deque
import numpy as np

HERE = os.path.expanduser('~/VPT-Framework/src/ros2_ws/src/vpt_gaze_bridge')
spec = importlib.util.spec_from_file_location('gbn', os.path.join(HERE, 'gaze_bridge_node.py'))
gbn = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gbn)

class FakeLogger:
    def __init__(self): self.msgs = []
    def warn(self, m): self.msgs.append(('warn', m))
    def info(self, m): self.msgs.append(('info', m))

class Stub:
    """compute_frontality_deg가 실제로 쓰는 상태만 흉내낸다."""
    def __init__(self):
        self._frontality_cos_buf = deque(maxlen=gbn.FRONTALITY_SIGN_CHECK_N)
        self._frontality_sign = 1.0
        self._frontality_sign_checked = False
        self._logger = FakeLogger()
    def get_logger(self): return self._logger

f = gbn.GazeBridgeNode.compute_frontality_deg

def rot_y(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])

def mat4(R):
    M = np.eye(4); M[:3, :3] = R
    return M.flatten().tolist()

fails = []
def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (('  | ' + detail) if detail else ''))
    if not cond: fails.append(name)

print('\n[1] 광축 위에서 카메라를 똑바로 마주봄 -> 0°')
s = Stub()
d = f(s, mat4(np.eye(3)), np.array([0.0, 0.0, 1.5]))
check('0°', abs(d - 0.0) < 1e-6, f'{d:.3f}°')

print('\n[2] 광축 위에서 얼굴만 30°/60° 돌림 -> 그대로 30°/60°')
for ang in (30.0, 60.0):
    s = Stub()
    d = f(s, mat4(rot_y(ang)), np.array([0.0, 0.0, 1.5]))
    check(f'{ang:.0f}°', abs(d - ang) < 1e-6, f'{d:.3f}°')

print('\n[3] 머리가 광축에서 벗어나면 광축 기준과 달라진다 (이 구현의 핵심)')
head_off = np.array([0.5, 0.0, 1.5])          # 오른쪽으로 0.5m 벗어남 -> 광축과 약 18.4°
s = Stub()
d_ours = f(s, mat4(np.eye(3)), head_off)      # 얼굴은 카메라 광축과 나란히 정면
off_axis_deg = np.degrees(np.arctan2(0.5, 1.5))
check('광축 기준(0°)이 아니라 실제 편위각을 반영',
      abs(d_ours - off_axis_deg) < 1e-6,
      f'ours={d_ours:.2f}°, 머리 편위={off_axis_deg:.2f}°, 광축기준이면 0.00°')
s = Stub()
d_facing = f(s, mat4(rot_y(off_axis_deg)), head_off)    # 얼굴을 카메라 쪽으로 돌리면
check('카메라를 실제로 마주보면 0°', abs(d_facing) < 1e-6, f'{d_facing:.4f}°')

print('\n[4] 부호 자동 판별: 전방축이 반대인 데이터 -> 경고 + 보정')
s = Stub()
flipped = mat4(rot_y(180.0))   # 얼굴이 카메라 반대쪽을 향하는 셈 -> cos 음수
out = [f(s, flipped, np.array([0.0, 0.0, 1.5])) for _ in range(gbn.FRONTALITY_SIGN_CHECK_N + 5)]
check('부호 보정 트리거됨', s._frontality_sign == -1.0, f'sign={s._frontality_sign}')
check('경고 로그 1회', sum(1 for lv, _ in s._logger.msgs if lv == 'warn') == 1,
      str([m for lv, m in s._logger.msgs if lv == 'warn'][:1]))
check('보정 후 0°로 수렴', abs(out[-1]) < 1e-6, f'{out[-1]:.4f}°')

print('\n[5] 부호가 정상이면 보정 안 하고 info 1회')
s = Stub()
for _ in range(gbn.FRONTALITY_SIGN_CHECK_N + 5):
    f(s, mat4(np.eye(3)), np.array([0.0, 0.0, 1.5]))
check('sign 유지', s._frontality_sign == 1.0, f'sign={s._frontality_sign}')
check('경고 없음', not any(lv == 'warn' for lv, _ in s._logger.msgs))
check('info 1회', sum(1 for lv, _ in s._logger.msgs if lv == 'info') == 1)

print('\n[6] 잘못된 입력 방어')
s = Stub()
check('head_cam=None -> None', f(s, mat4(np.eye(3)), None) is None)
check('head_cam=원점 -> None', f(s, mat4(np.eye(3)), np.zeros(3)) is None)

print('\n[7] 파일럿 기하 재현 (results_summary.md: 편위 22.6° / 87.2°)')
s = Stub()
d1 = f(s, mat4(rot_y(22.6)), np.array([0.0, 0.0, 1.0]))
s = Stub()
d2 = f(s, mat4(rot_y(87.2)), np.array([0.0, 0.0, 1.0]))
w1, w2 = np.cos(np.radians(d1)), np.cos(np.radians(d2))
check('두 각도 재현', abs(d1 - 22.6) < 1e-6 and abs(d2 - 87.2) < 1e-6, f'{d1:.1f}°, {d2:.1f}°')
check('가중치 비 18배 이상 (정면 쪽이 지배)', w1 / w2 > 18, f'w={w1:.3f} vs {w2:.3f} (비 {w1/w2:.1f}배)')

print('\n' + ('실패: ' + ', '.join(fails) if fails else '전체 통과'))
sys.exit(1 if fails else 0)
