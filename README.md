# SynchroBots — 스마트팩토리 통합 자동화 시스템

컨베이어로 이송되는 전자 모듈(ESP32, L298N, MB102)을 **AI 비전으로 분류·품질검사**하고, 불량품은 배출, 정상품은 **로봇팔이 집어 AMR에 적재**, AMR이 **자율주행으로 목적지까지 운송**하는 전 공정을 자동화한 스마트팩토리 프로젝트입니다.

핵심은 **OPC UA 표준 프로토콜 기반의 장비 통합**입니다. PLC(래더/Modbus), AMR(ROS1), 로봇팔(Python/PyTorch), 웹 관제(Flask)라는 서로 다른 스택의 장비들이 중앙 OPC UA 서버를 통해 JSON 메시지로 통신하며, 어떤 장비도 다른 장비를 직접 호출하지 않는 이벤트 기반 구조로 전체 사이클이 자동으로 순환합니다.

> 이 저장소(SynchroBots_DOC)는 프로젝트의 **문서 저장소**입니다. 소스 코드는 아래 저장소 구성의 링크를 참고하세요.

## 저장소 구성

| 저장소 | 역할 | 기술 스택 |
|---|---|---|
| [SynchroBots_PLC](https://github.com/gunchung78/SynchroBots_PLC) | **통신 허브 + 현장 설비 제어.** 중앙 OPC UA 서버(12개 인터페이스), XG5000 컨베이어 래더 로직, Modbus RTU↔OPC UA 게이트웨이, 배출 서보(Arduino), 구조물 SOLIDWORKS 설계 | Python(asyncua, pymodbus), XG5000, Arduino, SOLIDWORKS |
| [SynchroBots_WEB](https://github.com/gunchung78/SynchroBots_WEB) | **통합 관제 + AI 비전 검사.** Flask 웹서버 + OPC UA 구독 워커 2-프로세스 구조, ResNet50 분류 + PatchCore 이상탐지, 대시보드/제어패널/비전로그, 공정 자동화 오케스트레이션 | Flask, SQLAlchemy(MariaDB), PyTorch, OpenCV, asyncua, Chart.js |
| [SynchroBots_AMR](https://github.com/gunchung78/SynchroBots_AMR) | **자율주행 물류 로봇.** ROS1 Noetic 기반 gmapping/AMCL/move_base 내비게이션, ArUco 마커 정밀 도킹, OPC UA 미션 오케스트레이터, 미션·상태 DB 로깅 | ROS1, C++/Python, OpenCV(ArUco), YDLidar, MySQL |
| [SynchroBots_RobotArm](https://github.com/gunchung78/SynchroBots_RobotArm) | **비전 유도 픽앤플레이스.** OpenCV(이진 마스킹 + minAreaRect) 기반 파지 각도 산출, 분류+잔차 회귀 AI 각도 모델, 빨간 구역 탐지 Place 보정, OPC UA 미션 수신/보고 | Python(pymycobot, asyncua), PyTorch, OpenCV, ROS2 |
| [SynchroBots_DOC](https://github.com/gunchung78/SynchroBots_DOC) | 프로젝트 문서 (본 저장소) — WBS, 기능·인터페이스 정의서, 단계별 산출물, 회의록, 발표자료 | — |

## 시스템 아키텍처

```
                        ┌────────────────────────────────────┐
                        │        Web 관제 (Flask + MariaDB)   │
                        │  대시보드 · 제어패널 · 비전로그        │
                        │  AI 비전 검사 (ResNet50 + PatchCore) │
                        └──────┬─────────────────────▲───────┘
                     OPC UA 워커│(구독→웹훅)           │REST/SSE
                               ▼                     │
      ┌─────────────────────────────────────────────────────────┐
      │           중앙 OPC UA Server  (AMR/PLC/ARM/IMG 노드)      │
      │           write_* 메서드 호출 → read_* 노드 구독 중계       │
      └───┬──────────────────┬───────────────────┬──────────────┘
          │                  │                   │
   ┌──────▼──────┐    ┌──────▼──────┐     ┌──────▼──────────┐
   │ PLC Gateway │    │  Robot Arm  │     │      AMR        │
   │ (Modbus RTU)│    │ MyCobot 320 │     │  MyAGV 2023 Pi  │
   └──────┬──────┘    │ 비전 Pick &  │     │ ROS1 자율주행     │
   ┌──────▼──────┐    │ Place       │     │ LiDAR SLAM/AMCL │
   │ XG5000 PLC  │    └─────────────┘     │ ArUco 정밀 도킹  │
   │ 컨베이어·센서 │                        └─────────────────┘
   │ 배출 서보    │
   └─────────────┘
```

## 공정 흐름

1. **감지** — 컨베이어 포토센서 1이 모듈 감지 → PLC 접점 → Modbus 게이트웨이가 OPC UA로 전파
2. **AI 검사** — 웹 서버가 신호를 받아 카메라 10프레임 캡처 → ResNet50으로 모듈 분류(3종) → PatchCore 메모리뱅크로 이상 점수 산출 → PASS/REJECT 판정
3. **분기** — NG면 PLC 코일 펄스 → 서보가 불량품을 컨베이어 밖으로 배출, OK면 계속 이송
4. **픽앤플레이스** — 포토센서 2 감지 시 컨베이어 정지 → 로봇팔이 촬영 후 `minAreaRect`로 잡는 각도 산출 → Pick → AMR 상판의 빨간 적재 구역을 비전으로 찾아 Place
5. **운송** — 적재 완료 이력에서 방문 스테이션 목록을 역산해 AMR에 전달 → AMR이 LiDAR SLAM 자율주행 + ArUco 도킹으로 각 스테이션 순회
6. **관제** — 전 과정의 상태·이벤트·이미지가 DB에 기록되고 웹 대시보드에서 실시간 모니터링, 긴급정지 등 원격 제어

## 하드웨어

| 장비 | 모델 |
|---|---|
| AMR | Elephant Robotics MyAGV 2023 Pi (Raspberry Pi 4, ROS1 Noetic, YDLidar X2) |
| Robot Arm | Elephant Robotics MyCobot 320 M5 (전기 그리퍼) |
| PLC | LS ELECTRIC XG5000 계열 + 포토센서 2, 컨베이어 모터, HMI |
| 비전 | USB 웹캠 2대 (검사용 / 픽업 위치 인식용) |
| 배출 장치 | Arduino + 서보모터 |
| 구조물 | 컨베이어 기둥, AGV 적재함, 로봇암·센서·카메라 거치대 등 SOLIDWORKS 설계 후 3D 프린팅(PLA) 제작 |

검사 대상 워크피스는 ESP32 개발보드, L298N 모터드라이버, MB102 전원모듈 3종입니다.

## 통신 구조

**OPC UA 인터페이스 설계** — 모든 인터페이스는 `write_*` 메서드(명령 호출)와 `read_*` 변수 노드(구독 수신)의 쌍으로 구성됩니다. 장비가 메서드를 호출하면 서버가 JSON을 검증해 대응 노드에 쓰고, 그 노드를 구독하는 장비가 데이터 변경 알림으로 받아가는 중계 구조입니다. 처리 후 3초 뒤 노드를 자동으로 `"Ready"`로 리셋해 중복 처리를 방지합니다.

| 그룹 | 인터페이스 | 용도 |
|---|---|---|
| AMR | `amr_go_move` / `amr_go_positions` / `amr_mission_state` | 이동 명령, 목적지 리스트, 미션 상태 보고 |
| PLC | `conveyor_sensor_check` / `robotarm_sensor_check` / `ok_ng_value` / `ready_state` | 센서 감지 신호, 품질 판정, 컨베이어 재가동 |
| ARM | `send_arm_json` / `arm_go_move` / `arm_place_single` / `arm_place_completed` | 비전 결과·이미지 보고, 픽업 명령, 적재 완료 보고 |
| IMG | `send_arm_img` | JPG 이미지 전송 (ByteString) |

**하위 프로토콜** — OPC UA를 지원하지 않는 PLC는 PC 상주 게이트웨이가 Modbus RTU로 폴링/펄스 제어하며 양방향 번역합니다. 이미지는 224×224 JPEG를 base64로 인코딩해 OPC UA String 인자에 실어 전송합니다.

## AI 구성

| 용도 | 방법론 |
|---|---|
| 모듈 분류 | ResNet50 (fc → 3클래스), 10프레임 최빈값 투표 |
| 품질 이상탐지 | **PatchCore** — ResNet50 layer1~3 패치 임베딩 + 클래스별 정상 메모리뱅크 최근접 이웃 거리. 초기 Autoencoder 재구성 방식에서 정확도 문제로 전환 |
| 파지 각도(Rz) | OpenCV `minAreaRect` + AI(17-bin 분류 + 잔차 회귀) 0.8:0.2 앙상블 |
| Place 위치 보정 | HSV 빨간 구역 탐지 → 무게중심 → 픽셀-mm 선형 캘리브레이션 |

## 설계 포인트

- **이벤트 기반 통합** — 장비 간 직접 호출 없이 `OPC UA 노드 변경 → 구독 → 웹훅 → 다음 명령`의 고리로 전체 사이클이 순환. 장비 추가/교체 시 노드 매핑만 수정하면 되는 확장 구조
- **프로세스 분리** — 웹서버와 OPC UA 워커를 분리하고 HTTP 웹훅으로만 연결해 독립 재시작 가능, 워커에 자동 재접속 루프 내장
- **관심사 분리** — 로봇팔의 비전 노드는 로봇을 움직이지 않고, 좌표 산출과 구동을 분리. 제어용 경량 토픽과 로깅용 이미지 토픽도 분리
- **멱등성/중복 방지** — 명령 노드 자동 리셋, AMR 동일 명령 재수신 시 재주행 방지, ArUco 도킹 중복 호출 차단
- **현장 캘리브레이션 워크플로** — 로봇팔 티칭 모드(서보 해제 → 수동 자세 → 좌표 저장), 픽셀-mm 변환 계수 수집용 캡처 도구 내장
- **전 과정 감사 추적** — 모든 제어 명령이 출처(WEB/API), 트리거 이벤트, 결과와 함께 DB에 기록

---

# 문서 안내

프로젝트는 **1차 → 2차 → 최종** 3단계로 진행되었으며, 문서는 단계별 산출물과 공통 관리 문서로 구성됩니다.

```
SynchroBots_DOC
├── 1차 프로젝트/        # AGV 주행 검증 단계
│   ├── agv/                # myAGV 조이스틱 제어, SLAM 맵(map.pgm/yaml)
│   ├── AI/                 # 데이터 증강 스크립트, ResNet50·YOLO 학습 노트북
│   ├── PT/                 # 1차 발표자료
│   ├── 회의록/
│   ├── 1차 프로젝트 평가.txt
│   └── AGV_성공 영상_압축.mp4
├── 2차 프로젝트/        # PLC 공정 제어 단계
│   ├── PLC/                # XG5000 프로젝트, HMI(TDS) 프로젝트
│   ├── PT/                 # 2차 발표자료
│   └── 회의록/
├── 최종 프로젝트/       # 전체 통합 단계
│   ├── Final/              # 최종 발표자료, 시연 영상(컨베이어·로봇팔)
│   ├── PLC/                # OPC UA 서버 스냅샷
│   ├── WEB/                # 아키텍처·DB 명세·화면 설계 등 WEB 파트 산출물
│   ├── 제안서/             # 파트별 제안 발표자료 (SmartFactory/Monitoring/PLC/Vision/RobotArm/AMR)
│   ├── 회의록/
│   └── Flow.txt            # 최종 시스템 구성·공정 흐름 설계 문서
├── WBS/                 # 일정 관리 (날짜별 버전)
├── 기능정의서/           # AMR·WEB 기능 정의서, 인터페이스 정의서
├── 모델링/               # AGV 적재함·컨베이어 구조물 STL/G-code
└── 기타/                 # WEB 흐름도, 발표 피드백, 부품 구매목록, 개발 메모
```

## 주요 문서

| 문서 | 위치 | 내용 |
|---|---|---|
| 시스템 설계 | `최종 프로젝트/Flow.txt` | 전체 구성 요소, 하드웨어 사양, 5단계 공정 흐름 정의 |
| 인터페이스 정의서 | `기능정의서/SynchroBots_인터페이스 정의서_251126.xlsx` | OPC UA 인터페이스(AMR/PLC/ARM/IMG) 명세 |
| 기능 정의서 | `기능정의서/` | AMR·WEB 파트별 기능 명세 |
| DB 설계 | `최종 프로젝트/WEB/SynchroBots_테이블명세서.xlsx`, `DB 테이블 구조 설명.docx` | 테이블 구조와 설명 |
| 아키텍처 문서 | `최종 프로젝트/WEB/아키텍처 & 프로세스 흐름 설명 정리.docx` | WEB 파트 시퀀스 다이어그램 포함 |
| 최종 발표자료 | `최종 프로젝트/Final/SynchroBots_PT.pptx` | 시연 영상 포함 최종 발표 |
| WBS | `WBS/` | 프로젝트 일정 (251024 → 251120 버전 이력) |
