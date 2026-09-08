#!/usr/bin/env python3
"""GazeTR 각도 오차 정량 평가 — Pupil Core를 ground-truth 신뢰도 검증용으로 쓰는
"고정 타겟" 프로토콜(한국인공지능학술대회 2p 논문용, 2026-08-31 설계).

방법론: 벽에 카메라(D455/Kinect) 기준 3D 위치를 실측한 타겟을 여러 개 붙여두고,
피험자가 순서대로 응시한다. 각 트라이얼(타겟 하나를 보는 구간)에서:
  1. GazeTR 원본 벡터(gaze_bridge_node.py가 새로 발행하는 {camera}/gazetr_raw,
     카메라 로컬 프레임, EMA 스무딩/world 변환 전)의 평균을 구한다.
  2. "진짜" 시선 방향 = normalize(타겟 위치 - 머리 위치). 머리 위치는 같은 파이프라인의
     {camera}/head_position_cam(깊이 기반 추정, 카메라 로컬 프레임 — /head_position은 SLAM
     map 프레임으로 변환된 값이라 카메라 기준 실측 타겟 위치랑 못 섞어 씀)을 재사용한다.
  3. 두 벡터 사이의 각도(도)가 그 트라이얼의 오차다.

Pupil Core는 이 각도 계산에 직접 들어가지 않는다 — 대신 각 트라이얼 구간에서 피험자가
실제로 그 타겟을 정확히 보고 있었는지(컴플라이언스)를 confidence로 확인하는 데 쓴다.
이렇게 하면 Pupil Core ↔ D455/Kinect 사이의 외부 캘리브레이션을 새로 잡을 필요가 없다
(둘 다 "카메라 기준 실측 타겟 위치"라는 같은 물리적 기준에 대해서만 각자 비교됨).

## 사용 절차

1. 타겟보드 설치 후 각 타겟의 3D 위치(카메라 기준, 미터)를 줄자로 실측 —
   trial_config.yaml에 기입 (예시는 이 파일 맨 아래 `--write-example-config` 참고).
2. 실험 중 아래 두 토픽을 CSV로 녹화(피험자가 트라이얼 순서대로 타겟을 보는 동안):
     ros2 topic echo /kinect/gazetr_raw --csv > gazetr_raw.csv    (또는 /d455/...)
     ros2 topic echo /kinect/head_position_cam --csv > head_position.csv
   두 CSV 다 timestamp 컬럼이 있어야 함(기본 --csv 출력에 포함됨).
3. Pupil Player로 Pupil Core 녹화본을 열어 gaze_positions.csv로 export.
4. 트라이얼별 시작/끝 시각(유닉스 타임스탬프, 실험 중 "타겟 1 시작 시각" 식으로 메모해두거나
   Pupil Core annotation 기능으로 남겨도 됨)을 trial_config.yaml에 기입.
5. 실행:
     python scripts/eval_gazetr_accuracy.py trial_config.yaml \
         --gazetr-csv gazetr_raw.csv --head-csv head_position.csv \
         --pupil-csv gaze_positions.csv
"""
import argparse
import csv
import sys

import numpy as np
import yaml


def angular_error_deg(v1, v2):
    """두 3D 벡터 사이의 각도(도). 둘 다 자동으로 정규화한다."""
    v1 = np.asarray(v1, dtype=np.float64)
    v2 = np.asarray(v2, dtype=np.float64)
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-9 or n2 < 1e-9:
        raise ValueError("영벡터는 각도를 정의할 수 없음")
    cosine = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def load_vector_csv(path):
    """`ros2 topic echo <Vector3Stamped 또는 PointStamped topic> --csv`로 뽑은 CSV를 읽는다.
    반환: (timestamps(초, float 배열), xyz(N,3) 배열).
    ros2 --csv 기본 컬럼 순서: header.stamp.sec, header.stamp.nanosec, header.frame_id,
    (point 또는 vector).x, .y, .z — Vector3Stamped/PointStamped 둘 다 동일한 위치."""
    ts, xyz = [], []
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or not row[0].strip().lstrip('-').isdigit():
                continue  # 헤더 줄이나 빈 줄 skip
            sec, nsec = int(row[0]), int(row[1])
            x, y, z = float(row[3]), float(row[4]), float(row[5])
            ts.append(sec + nsec * 1e-9)
            xyz.append((x, y, z))
    if not ts:
        raise ValueError(f"{path}에서 유효한 행을 못 찾음 — --csv로 뽑은 파일이 맞는지 확인")
    return np.array(ts), np.array(xyz)


def load_pupil_confidence_csv(path):
    """Pupil Player가 export한 gaze_positions.csv. 반환: (world_timestamp 배열, confidence 배열).
    컬럼명이 버전에 따라 다를 수 있어 DictReader로 유연하게 찾는다."""
    ts, conf = [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = row.get('world_timestamp') or row.get('gaze_timestamp') or row.get('timestamp')
            c = row.get('confidence')
            if t is None or c is None:
                continue
            ts.append(float(t))
            conf.append(float(c))
    return np.array(ts), np.array(conf)


def window_mean(ts, values, t_start, t_end):
    mask = (ts >= t_start) & (ts <= t_end)
    n = int(mask.sum())
    if n == 0:
        return None, 0
    return values[mask].mean(axis=0), n


def evaluate(config_path, gazetr_csv, head_csv, pupil_csv=None,
             min_confidence=0.6, out_csv=None):
    with open(config_path) as f:
        config = yaml.safe_load(f)
    camera_origin = np.array(config.get('camera_origin_m', [0.0, 0.0, 0.0]), dtype=np.float64)
    trials = config['trials']

    gazetr_ts, gazetr_xyz = load_vector_csv(gazetr_csv)
    head_ts, head_xyz = load_vector_csv(head_csv)
    pupil_ts = pupil_conf = None
    if pupil_csv:
        pupil_ts, pupil_conf = load_pupil_confidence_csv(pupil_csv)

    results = []
    for trial in trials:
        name = trial['name']
        t0, t1 = float(trial['t_start']), float(trial['t_end'])
        target_pos = np.array(trial['target_pos_m'], dtype=np.float64) - camera_origin

        gaze_mean, n_gaze = window_mean(gazetr_ts, gazetr_xyz, t0, t1)
        head_mean, n_head = window_mean(head_ts, head_xyz, t0, t1)

        row = {'trial': name, 'n_gazetr_samples': n_gaze, 'n_head_samples': n_head}

        if pupil_ts is not None:
            mask = (pupil_ts >= t0) & (pupil_ts <= t1)
            n_pupil = int(mask.sum())
            mean_conf = float(pupil_conf[mask].mean()) if n_pupil else float('nan')
            n_low_conf = int((pupil_conf[mask] < min_confidence).sum()) if n_pupil else 0
            row['pupil_mean_confidence'] = round(mean_conf, 3)
            row['pupil_low_confidence_frac'] = (
                round(n_low_conf / n_pupil, 3) if n_pupil else float('nan'))
            row['compliant'] = bool(n_pupil and mean_conf >= min_confidence)
        else:
            row['compliant'] = None

        if gaze_mean is None or head_mean is None:
            row['angular_error_deg'] = None
            print(f"[경고] {name}: 이 구간에 gazetr={n_gaze}개, head={n_head}개 샘플 — 오차 계산 불가")
        else:
            true_dir = target_pos - head_mean
            row['angular_error_deg'] = round(angular_error_deg(gaze_mean, true_dir), 2)

        results.append(row)

    valid_errors = [r['angular_error_deg'] for r in results if r['angular_error_deg'] is not None]
    print(f"\n{'trial':<12} {'err(deg)':>9} {'n_gaze':>7} {'n_head':>7} {'compliant':>10}")
    for r in results:
        err = f"{r['angular_error_deg']:.2f}" if r['angular_error_deg'] is not None else "N/A"
        print(f"{r['trial']:<12} {err:>9} {r['n_gazetr_samples']:>7} "
              f"{r['n_head_samples']:>7} {str(r['compliant']):>10}")

    if valid_errors:
        arr = np.array(valid_errors)
        print(f"\n전체: n={len(arr)}  평균={arr.mean():.2f}도  "
              f"표준편차={arr.std():.2f}도  최대={arr.max():.2f}도  최소={arr.min():.2f}도")
    else:
        print("\n[경고] 유효한 트라이얼이 하나도 없음")

    if out_csv:
        fieldnames = list(results[0].keys()) if results else []
        with open(out_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)
        print(f"\n저장: {out_csv}")

    return results


EXAMPLE_CONFIG = """\
# 카메라(D455 또는 Kinect) 렌즈 위치를 원점(0,0,0)으로 잡아도 되고, 다른 고정 기준점을
# 잡고 camera_origin_m으로 상대위치를 빼줘도 됨. 전부 미터 단위.
camera_origin_m: [0.0, 0.0, 0.0]

trials:
  - name: target_1
    target_pos_m: [-0.5, 0.2, 1.5]   # 카메라 기준 [x, y, z], 줄자로 실측
    t_start: 1787990000.0            # 유닉스 타임스탬프(초) — 실험 중 메모한 시각
    t_end: 1787990003.0
  - name: target_2
    target_pos_m: [0.0, 0.2, 1.5]
    t_start: 1787990010.0
    t_end: 1787990013.0
  - name: target_3
    target_pos_m: [0.5, 0.2, 1.5]
    t_start: 1787990020.0
    t_end: 1787990023.0
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", nargs="?", help="trial_config.yaml")
    ap.add_argument("--gazetr-csv")
    ap.add_argument("--head-csv")
    ap.add_argument("--pupil-csv", default=None)
    ap.add_argument("--min-confidence", type=float, default=0.6)
    ap.add_argument("--out-csv", default=None, help="트라이얼별 결과를 이 CSV로도 저장")
    ap.add_argument("--write-example-config", metavar="PATH",
                     help="예시 trial_config.yaml을 이 경로에 써주고 종료")
    ap.add_argument("--self-test", action="store_true",
                     help="합성 데이터로 angular_error_deg 자체 검증만 하고 종료")
    args = ap.parse_args()

    if args.write_example_config:
        with open(args.write_example_config, "w") as f:
            f.write(EXAMPLE_CONFIG)
        print(f"예시 config 작성: {args.write_example_config}")
        return

    if args.self_test:
        assert abs(angular_error_deg([1, 0, 0], [1, 0, 0])) < 1e-9
        assert abs(angular_error_deg([1, 0, 0], [0, 1, 0]) - 90.0) < 1e-9
        assert abs(angular_error_deg([1, 0, 0], [-1, 0, 0]) - 180.0) < 1e-9
        assert abs(angular_error_deg([2, 0, 0], [3, 0, 0])) < 1e-9  # 스케일 무관
        print("[self-test] OK — angular_error_deg 정상")
        return

    if not (args.config and args.gazetr_csv and args.head_csv):
        ap.error("config, --gazetr-csv, --head-csv는 필수(또는 --self-test / "
                  "--write-example-config만 단독 사용)")

    evaluate(args.config, args.gazetr_csv, args.head_csv,
              pupil_csv=args.pupil_csv, min_confidence=args.min_confidence,
              out_csv=args.out_csv)


if __name__ == "__main__":
    sys.exit(main())
