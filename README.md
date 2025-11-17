# 국립한밭대학교 컴퓨터공학과 DfX팀

**팀 구성**
- 30251270 유태원
- 30251268 송찬호
- 20202066 안다은
- 20201737 백민우
- 20222278 박동현

## Project Background
깻잎은 연중 재배되는 대표적인 엽채류로, 현재까지 수확 공정이 숙련 인력의 수작업에 의존하고 있습니다.
이로 인해 노동 강도, 생산성 저하, 품질 편차 등의 문제가 발생하고 있습니다.

본 프로젝트는 광센서 기반의 줄기 탐지 기술과 와이어 구동 절단 메커니즘을 결합한 엔드이펙터를 설계하여
잎 손상 없는 자동 수확 시스템을 구현하는 것을 목표로 합니다.

<img width="572" height="216" alt="image" src="https://github.com/user-attachments/assets/9f8e42d5-17f9-4d9b-bf92-9dda0b1c1dfa" />

## System Model

### 1) 전체 구조 개요
본 시스템은 비전 인식부, 로봇 제어부, 엔드이펙터 구동부의 세 가지 핵심 모듈로 구성됩니다.
RGB-D 카메라로 획득한 데이터를 기반으로 YOLOv11-OBB 모델이 깻잎의 위치와 자세를 인식하고,
Jetson Orin Nano에서 연산 후 Neuromeka Indy7 로봇팔과 OpenCR 제어기로 명령을 전달합니다.

<img width="375" height="235" alt="image" src="https://github.com/user-attachments/assets/910e14fe-15c2-4f67-848c-790b02b0a903" />

**엔드이펙터 구동부**: 포토센서(줄기), 마이크로 포토센서(잎자루), Dynamixel 모터 + 와이어 메커니즘, OpenCR 제어

**비전 인식부**: RGB-D 카메라(예: RealSense)로 컬러/깊이 영상 수집 → YOLOv11-OBB로 잎 위치·회전각 검출

**로봇 제어부**: Jetson(Orin Nano)이 인식 결과를 로봇 좌표계로 변환해 6DoF 로봇(예: Indy7)을 제어

### 2) 작동 절차
깻잎 자동 수확 시스템의 전체 동작 절차는 5단계로 구성되어 있습니다

<img width="550" height="500" alt="image" src="https://github.com/user-attachments/assets/8fedd7fa-a91c-4ff3-8abe-4b0ab232d186" />

**1. 카메라 촬영**: YOLOv11-OBB로 인식된 깻잎의 3D 좌표를 계산 후 좌하단→우상단 순서로 수확 경로를 설정

**2. 측면으로 이동**: 생장점을 보호하기 위해 잎보다 약 6cm 낮은 위치에서 접근

**3. 줄기로 진입**: 포토센서 감지 신호에 따라 줄기를 안정적으로 파지

**4. 잎자루로 접근**: 마이크로 포토센서가 잎자루 통과를 감지하면 절단 명령 대기

**5. 절단 및 파지**: 와이어 메커니즘이 작동해 절단과 파지를 동시에 수행

### 3) 엔드이펙터 제작

#### 구조

<img width="450" height="400" alt="image" src="https://github.com/user-attachments/assets/1bc6bb1e-8311-4ed6-80ff-037ea94b565d" />

**1. 센서 배치**: 절단부에 포토센서(줄기)와 마이크로 포토센서(잎자루) 탑재  
**2. 구동부**: Dynamixel 모터의 회전을 **와이어 인장력**으로 변환, 절단부와 연동  
**3. 제어부**: OpenCR이 센서 신호 통합·모터 제어, RS-485/TTL 컨버터로 장거리 통신 안정화  
**4. 제작**: 3D 프린팅 기반 경량·모듈형 본체, 카메라 시야 간섭 최소화 배치

#### 작동 원리

<img width="600" height="300" alt="image" src="https://github.com/user-attachments/assets/7af06fee-38b2-4eb7-be2f-7c2a5bfdb848" />

**1. 초기 상태**: 스프링 복원력으로 집게부 열림  
**2. 파지 단계**: 줄기 감지 → 와이어 인장 → 집게부 닫힘(줄기 고정)  
**3. 절단 단계**: 장력 증가 시 집게 한쪽 면과 벽면이 맞물려 **전단력** 발생 → 잎자루 절단 & 파지 동시 수행  
**4. 복귀 단계**: 장력 해제 → 스프링 복원 → 원위치


### 4) 객체 매칭 및 자세 추정

<img width="224" height="168" alt="image" src="https://github.com/user-attachments/assets/becceb42-eb3f-4744-96ec-2365391aef23" />

- 중심 잎의 양쪽 **긴 변(mid-edge)** 중점을 기준으로 주변 수확용 잎의 경계 상자(OBB)까지의 거리를 계산 
- 각 중점에서 가장 가까운 수확 잎을 선택하여 **1개의 중심 잎 ↔ 최대 2개의 수확 잎** 관계를 형성
- 중복 매칭 발생 시 더 짧은 거리의 매칭만 유지하여 **정확한 leaf entity 구성**을 보장

<img width="200" height="171" alt="image" src="https://github.com/user-attachments/assets/f4d58d30-62b4-4ee0-9ba4-3cdae520ce72" />

- 매칭된 수확 잎 쌍의 OBB에서 **가장 가까운 변의 중점(midpoint)** 을 계산하고,  
  두 점을 잇는 선분의 **중심(center)** 을 줄기 위치로,
  선분의 **기울기(angle)** 을 잎의 방향(orientation)으로 정의
- 이 정보를 로봇 제어부에서 활용하여 **엔드이펙터의 진입 위치 및 각도 보정**을 수행

### 5) 좌표 변환 및 로봇 접근

<img width="265" height="255" alt="image" src="https://github.com/user-attachments/assets/1a44b5b8-aa97-4e32-8b67-eb2fbd425445" />

- RGB-D 카메라의 픽셀 좌표를 실공간 3차원 좌표로 변환 
- 변환된 좌표를 이용해 로봇팔이 깻잎 줄기 위치와 방향에 정밀하게 접근하도록 제어  
- 깊이 정보와 카메라 내부 파라미터를 활용해 좌표를 환산
- 카메라–로봇 간 오프셋과 회전각을 보정하여 엔드이펙터 진입 위치를 계산

## Numerical Results
### YOLOv11-OBB
| Metric | Precision | Recall | mAP@0.5 | mAP@0.5:0.95 |
|:------:|:--------:|:-----:|:-------:|:------------:|
| Result | 0.977    | 0.983 | 0.994   | 0.864        |

### Table 2. Experimental results of the perilla harvesting process

|           | Step 1 | Step 2 | Step 3 | Result |
|-----------|:------:|:------:|:------:|:-----:|
| Trials    |   50   |   50   |   46   |  50   |
| Successes |   50   |   46   |   41   |  41   |
| Performance | 100% |  92%   |  89%   |  82%  |

## Conclusion

## Project Outcome

<img width="509" height="441" alt="image" src="https://github.com/user-attachments/assets/16bcf9e8-a5fc-4cb5-976a-39c90ab9a604" />

