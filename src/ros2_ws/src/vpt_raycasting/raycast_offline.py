"""
Ray Casting - Gaze Projection onto 3D Map (offline/standalone)

저장된 .ply 포인트클라우드에 대고 ray casting을 테스트하기 위한 오프라인 스크립트.
ROS2로 실행되는 실사용 버전은 raycasting_node.py 참고.
"""

import numpy as np
from scipy.spatial import KDTree
import struct
import time


def load_ply(filepath):
    print(f"PLY 파일 로딩 중: {filepath}")
    
    with open(filepath, 'rb') as f:
        # 헤더 읽기
        header_lines = []
        while True:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            header_lines.append(line)
            if line == 'end_header':
                break
        
        num_vertices = 0
        properties = []
        is_little_endian = True
        
        for line in header_lines:
            if line.startswith('element vertex'):
                num_vertices = int(line.split()[-1])
            elif line.startswith('property'):
                parts = line.split()
                properties.append((parts[1], parts[2]))
            elif 'binary_little_endian' in line:
                is_little_endian = True
            elif 'binary_big_endian' in line:
                is_little_endian = False
        
        print(f"포인트 수: {num_vertices:,}")
        
        type_info = {
            'float': ('f', 4), 'double': ('d', 8),
            'uchar': ('B', 1), 'char': ('b', 1),
            'int': ('i', 4), 'uint': ('I', 4),
            'short': ('h', 2), 'ushort': ('H', 2),
            'int32': ('i', 4), 'uint32': ('I', 4),
            'int8': ('b', 1), 'uint8': ('B', 1),
        }
        
        endian = '<' if is_little_endian else '>'
        prop_formats = []
        prop_names = []
        
        for ptype, pname in properties:
            fmt_char, size = type_info.get(ptype, ('f', 4))
            prop_formats.append((fmt_char, size))
            prop_names.append(pname)
        
        total_size = sum(s for _, s in prop_formats)
        print(f"레코드 크기: {total_size} bytes")
        
        x_idx = prop_names.index('x') if 'x' in prop_names else 0
        y_idx = prop_names.index('y') if 'y' in prop_names else 1
        z_idx = prop_names.index('z') if 'z' in prop_names else 2
        r_idx = prop_names.index('red') if 'red' in prop_names else None
        g_idx = prop_names.index('green') if 'green' in prop_names else None
        b_idx = prop_names.index('blue') if 'blue' in prop_names else None

        # 전체 데이터 한번에 읽기
        raw_data = f.read()
    
    # 읽을 수 있는 최대 포인트 수
    max_points = len(raw_data) // total_size
    actual_points = min(num_vertices, max_points)
    print(f"실제 읽을 포인트 수: {actual_points:,}")
    
    # numpy로 빠르게 파싱
    # 각 속성별 dtype 구성
    dt_list = []
    for (fmt_char, size), pname in zip(prop_formats, prop_names):
        np_type = {
            'f': np.float32, 'd': np.float64,
            'B': np.uint8, 'b': np.int8,
            'i': np.int32, 'I': np.uint32,
            'h': np.int16, 'H': np.uint16,
        }.get(fmt_char, np.float32)
        dt_list.append((pname, np_type))
    
    dtype = np.dtype(dt_list)
    
    print("numpy로 파싱 중...")
    data = np.frombuffer(raw_data[:actual_points * total_size], dtype=dtype)
    
    points = np.column_stack([
        data['x'].astype(np.float32),
        data['y'].astype(np.float32),
        data['z'].astype(np.float32)
    ])
    
    # NaN/Inf 제거
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]
    print(f"유효 포인트: {len(points):,} (제거: {actual_points - len(points):,})")
    
    colors = None
    if r_idx is not None:
        colors = np.column_stack([
            data['red'][valid],
            data['green'][valid],
            data['blue'][valid]
        ])
    
    print("로딩 완료!")
    return points, colors


def build_kdtree(points):
    print("KDTree 구축 중...")
    tree = KDTree(points)
    print("KDTree 구축 완료!")
    return tree


def ray_cast(origin, direction, points, kdtree, max_dist=5.0, step=0.05):
    direction = direction / np.linalg.norm(direction)
    threshold = 0.1

    t = 0.1
    while t < max_dist:
        ray_point = origin + t * direction
        dist, idx = kdtree.query(ray_point, k=1)
        if dist < threshold:
            return points[idx]
        t += step
    return None


def main():
    PLY_PATH = "/home/seng/.ros/rtabmap_cloud.ply"
    
    points, colors = load_ply(PLY_PATH)
    kdtree = build_kdtree(points)
    
    print("\n=== 포인트클라우드 통계 ===")
    print(f"X 범위: {points[:,0].min():.2f} ~ {points[:,0].max():.2f}")
    print(f"Y 범위: {points[:,1].min():.2f} ~ {points[:,1].max():.2f}")
    print(f"Z 범위: {points[:,2].min():.2f} ~ {points[:,2].max():.2f}")
    center = points.mean(axis=0)
    print(f"중심점: [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
    
    print("\n=== Ray Casting 테스트 ===")
    test_cases = [
        {"name": "정면",   "origin": center, "direction": np.array([0.0, 0.0, 1.0])},
        {"name": "왼쪽",   "origin": center, "direction": np.array([-0.5, 0.0, 0.866])},
        {"name": "오른쪽", "origin": center, "direction": np.array([0.5, 0.0, 0.866])},
        {"name": "위쪽",   "origin": center, "direction": np.array([0.0, 0.3, 0.954])},
    ]
    
    for case in test_cases:
        start = time.time()
        G_t = ray_cast(case["origin"], case["direction"], points, kdtree)
        elapsed = time.time() - start
        
        if G_t is not None:
            print(f"{case['name']}: G_t = [{G_t[0]:.3f}, {G_t[1]:.3f}, {G_t[2]:.3f}] ({elapsed*1000:.1f}ms)")
        else:
            print(f"{case['name']}: 교차점 없음")


if __name__ == "__main__":
    main()
