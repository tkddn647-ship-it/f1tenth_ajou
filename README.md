# F1TENTH Roboracer — 아주대학교 (29th F1TENTH Roboracer @ IFAC)

1/10 스케일 자율주행 레이싱카를 위한 ROS 2 스택입니다.
아주대학교는 이 대회에 **처음 출전해 본선에 진출**했습니다.

| 항목 | 내용 |
|------|------|
| 기간 | 2025.10 ~ 2026.08 |
| 플랫폼 | Jetson Xavier NX · RPLIDAR · EBIMU · ESP32 (하위 제어) |
| 소프트웨어 | ROS 2 (Python 3.10), C++ / Python, Cartographer SLAM |
| 결과 | 주행 속도 **5.0 → 6.0 m/s**, 학교 최초 출전 **본선 진출** |

---

## 내 역할 (권상우)

- **개발 환경 구축**: 참고할 코드 없이 Jetson에 Linux·ROS 2 환경을 직접 구성하고, 세팅 과정을 문서로 남겨 팀원에게 공유
- **노드 구조 설계**: 센서 드라이버 · 위치추정 · 주행 계획을 독립 패키지로 나눠 팀원별로 병렬 개발할 수 있게 구성
- **위치추정 튜닝**: Cartographer SLAM 적용, 라이다·IMU·차체 TF Tree 설계 (`tf_manager_cpp`, C++)
- **트러블슈팅**: 섞여 있던 고장 현상 3가지를 하나씩 분리해 해결 (아래 표)

## 해결한 문제

| 현상 | 판단 근거 | 조치 | 결과 |
|------|-----------|------|------|
| 배터리 전압이 흔들릴 때 조종 입력이 끊김 | 전압 변동 시점과 끊김 시점이 겹침 | 하위 제어 보드를 Arduino Nano → ESP32로 교체, 수동 제어 계층을 자율주행 계층과 분리 | 한쪽이 멈춰도 다른 쪽이 동작 |
| 위치추정 지연 | 연산 부족으로 판단 | Jetson Nano → Xavier NX | 연산 여유 확보 |
| 직선 구간에서 지도가 압축됨 | 특징이 적은 직선 복도에서 스캔 매칭이 엉뚱한 위치를 고름 | 라이다 측정 범위 40 → 22 m, 매칭 탐색 범위 10 → 4 cm로 축소 | 주행 속도 5.0 → 6.0 m/s |

> 현상이 여럿 섞여 있을 때는 한 번에 하나만 바꾸고 결과를 기록해야 원인이 갈린다는 것을 배웠습니다.

---

## 시스템 구조

```
[RPLIDAR] ─/scan──┐
[EBIMU]  ─/imu────┼─▶ localization_layer (Cartographer) ─▶ TF: map → odom → base_link
[휠속도·조향] ─────┘          ▲
                            tf_manager_cpp (센서 고정 TF, 휠 오도메트리)

[CSV 경로] ─▶ centerline_publisher ─/recommended_path (2 Hz)──┐
/scan ─────▶ static_obstacle_detector ─/static_obstacles ─────┼─▶ local_planner ─/local_path (20 Hz)─▶ Pure Pursuit ─/drive─▶ 차량
/scan ─────▶ fgm_node (Follow the Gap) ─/fgm_target ──────────┘
```

- 장애물이 멀면 추천 경로를 그대로 따르고, 가까우면(기본 0.8 m) FGM 회피 목표점을 경로 앞에 이어 붙입니다.
- 노드별 상세 동작과 파라미터는 [`src/race_pkg/ARCHITECTURE.md`](src/race_pkg/ARCHITECTURE.md)에 정리돼 있습니다.

## 패키지 구성

| 패키지 | 언어 | 역할 |
|--------|------|------|
| `sensor_layer` | — | 센서 드라이버 실행과 파라미터 (`config/sensor_params.yaml`) |
| `sllidar_ros2` | C++ | RPLIDAR 드라이버 (Slamtec 공개 드라이버, 원 라이선스 유지) |
| `ebimu_pkg` | Python | EBIMU IMU 드라이버 / 퍼블리셔 |
| `tf_manager_cpp` | C++ | 센서 고정 TF, 휠속도·조향 기반 오도메트리 TF |
| `localization_layer` | Lua / Python | Cartographer 매핑·위치추정 설정, 지도 자동 저장 |
| `race_pkg` | Python | 경로 발행, 정적 장애물 검출, FGM, 로컬 플래너 |
| `race_layer` | Python | 주행 스택 launch 묶음 |
| `sim_test` | Python | 하드웨어 없이 매핑·위치추정을 시험하는 시뮬레이션 패키지 |

`maps/` 에는 트랙 지도(`.pbstream`, `.png/.yaml`)와 경로 CSV가 들어 있습니다.

## 빌드와 실행

```bash
# 워크스페이스 루트에서
colcon build --symlink-install
source install/setup.bash

# 매핑
ros2 launch localization_layer cartographer_mapping_launch.py

# 주행 스택 (경로 발행 · 장애물 검출 · FGM · 로컬 플래너)
ros2 launch race_pkg f1tenth_drive.launch.py

# 하드웨어 없이 시험
ros2 launch sim_test sim_mapping.launch.py
```

> 일부 launch 파일의 지도·CSV 기본 경로가 개발 PC 절대 경로(`/home/tkddn647/test/maps/...`)로 되어 있습니다.
> 다른 환경에서는 launch 인자로 경로를 넘겨 주세요.

## 남은 과제

- launch 파일의 절대 경로를 패키지 상대 경로로 정리
- 대회에서 쓴 최종 파라미터 세트를 별도 config로 고정해 재현성 확보
