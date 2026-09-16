# MMH_HAND (핸드2) — 13자유도 로봇손 펌웨어

STM32G474VETx 기반, 4지(엄지/검지/중지/약지) 13자유도 로봇손의 모터 제어 펌웨어입니다.

> 이 폴더는 **PC-side 실행 패키지가 아니라 MCU 펌웨어**입니다. 현재는 명령/상태 전환을 ST-Link **Live Watch**로 디버거가 직접 poke하는 방식으로 제어하며, 통신 프로토콜은 아직 붙어있지 않습니다. 상위 시스템에서 제어하려면 §5 통신 방식을 먼저 읽어주세요.

---

## 1. 실행 흐름

```mermaid
flowchart TD
    A[HAL_Init / SystemClock_Config] --> B[MX_GPIO_Init / MX_DMA_Init\nMX_ADC1~3_Init / MX_USART2_Init\nMX_TIM1,3,4,15_Init]
    B --> C[ADC DMA 시작: adc1/2/3_buf]
    C --> D[모터드라이버 Sleep 해제\nGPIOE 3,4,5 = HIGH]
    D --> E[Hand_Sensor_Update\nHand_Motor_Init]
    E --> F[PWM 채널 13개 시작\nTIM1/TIM3/TIM4/TIM15]
    F --> G{{메인 루프 while(1)}}
    G --> H[Hand_Sensor_Update\nADC → 각도/전류]
    H --> I[system_state 상태머신\n0=자유 1=홈 4=그립시퀀스]
    I --> J[Hand_Motor_PID_Compute]
    J --> K[Hand_Motor_Drive\nGPIO 방향 + PWM 출력]
    K --> L[HAL_Delay 10ms]
    L --> G
```

제어 주기는 **10ms**(`HAL_Delay(10)`)입니다.

---

## 2. 모터 매핑 & 게인 테이블

`motors[14]` 배열의 인덱스 1~13을 사용합니다 (`motor_init_data[14]`, `Core/Src/hand_motor.c` 기준).

| 모터 | offset | Kp | Ki | Kd | PWM limit | dead zone | inv | adc_min | adc_max | init_angle | 방향핀 | PWM 채널 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| M1 | 1610 | 50 | 0 | 5 | 4000 | 150 | 1 | 1550 | 2700 | 90° | PC10 | TIM3_CH1 |
| M2 | 2090 | 200 | 0 | 3 | 4000 | 150 | 0 | 2090 | 3140 | 90° | PC11 | TIM3_CH2 |
| M3 | 2000 | 200 | 0 | 3 | 4000 | 150 | 0 | 2200 | 2900 | 50° | PC12 | TIM3_CH3 |
| M4 | 1450 | 200 | 0 | 3 | 4000 | 150 | 0 | 1600 | 2830 | 30° | PD0 | TIM3_CH4 |
| M5 | 1100 | 200 | 0 | 3 | 4000 | 150 | 0 | 1100 | 1680 | 25° | PD8 | TIM4_CH1 |
| M6 | 1950 | 200 | 0 | 3 | 4000 | 150 | 0 | 2100 | 3320 | 30° | PD9 | TIM4_CH2 |
| M7 | 1780 | 200 | 0 | 3 | 4000 | 150 | 0 | 1780 | 2450 | 30° | PD10 | TIM4_CH3 |
| M8 | 1650 | 200 | 0 | 3 | 5000 | 500 | 0 | 1550 | 2800 | 75° | PD11 | TIM4_CH4 |
| M9 | 1300 | 100 | 0 | 0 | 4000 | 150 | 1 | 0 | 4095 | 100° | PE12 | TIM1_CH1 |
| M10 | 1300 | 100 | 0 | 0 | 4000 | 150 | 0 | 0 | 4095 | -50° | PE15 | TIM1_CH2 |
| M11 | 1300 | 100 | 0 | 0 | 4000 | 150 | 0 | 0 | 4095 | 160° | PB10 | TIM1_CH3 |
| M12 | 1300 | 80 | 0 | 0 | 4000 | 150 | 1 | 0 | 4095 | 180° | PB11 | TIM1_CH4 |
| M13 | 0 | 100 | 0 | 0 | **5655**(최고) | 150 | 0 | 50 | 4000 | 180° | PB13 | TIM15_CH1 |

- M9~M12: 다회전 처리(`turn_count`), 전류센서(`raw_current_adc`) 별도 수집 — 코드 상으로는 **M9~M13(5채널, `adc3_buf`)** 까지 수집됩니다. (§8 참고 — 힘 제어 통합 미완료)
- M13: 별도 각도변환식(180°/3600pulse), `pwm_limit` 전 모터 중 최고치.
- 모터드라이버 Sleep 해제 핀: `GPIOE3/4/5` (부팅 시 `HAL_Delay(50)` 후 HIGH로 설정).

---

## 3. 제어 모드 (`control_mode`)

| 모드 | 동작 |
|---|---|
| 0 | Lock/Open, 자유모드 대기 |
| 1 | 즉시 목표각 PID 위치제어 |
| 2 | 램핑(궤적) 제어, step_size 0.8°/10ms |
| 3 | 상수힘 제어, PWM 2500 (M9~M12 전용) |
| 4 | 상수힘 제어, PWM 4000 (M9~M12 전용, 모터 발열 주의 — 짧게만 사용 권장) |

`system_state`(전역): `0`=자유모드(Live Watch 수동 조작 대기), `1`=홈모드(전 관절 초기각 고정), `4`=그립 시퀀스(§ 다음 절). **`2`, `3`, `5`는 아직 이식되지 않았습니다** — §8 참고.

---

## 4. 그립 시퀀스 (`system_state == 4`, `state4_step`)

| 스텝 | 트리거 | 동작 | M13 | M9~M12 |
|---|---|---|---|---|
| 0 | state4 진입 | M1~8 → 모드2, M9~12 → 모드1 설정 | 위치 유지 | 모드1 |
| 1 | 스텝0 직후 | M1,3,5,7 벌려서 물체 감싸기 (10°,20°,25°,50°) | 그대로 | 그대로 |
| 2 | \|error\|<4° 도달 | M9,11,12 즉시 / M10 500ms 지연 → 모드3 전환 | 그대로 | 모드3 진입 |
| 3 | 1000ms 경과 | M2,4,6,8 1차 움켜쥐기 (70°,70°,80°,40°) | 그대로 | 그대로 |
| 4 | \|error\|<4° 도달 | 2초 대기 | 그대로 | 그대로 |
| 5 | 2초 경과 | M2,4,6,8 재설정(30°,90°,90°,30°), 물체에 걸려도 계속 압박 | 그대로 | 그대로 |
| 6 | `bit_2` 플래그(디버거 수동 트리거) | M13이 목표각 50°까지 연속 회전, 동시에 M9~12 모드3→모드4 전환 | 목표각 이동 완료 | 모드4(고출력) |

---

## 5. 통신 방식

- **현재**: 없음. `system_state`, `bit_2` 등은 ST-Link **Live Watch로 디버거가 직접 poke**하는 방식으로 제어됩니다. `MX_USART2_UART_Init()`은 호출되지만 실제 수신 파싱 로직은 없습니다.

---

## 6. 빌드 방법

1. STM32CubeIDE로 이 폴더를 **Import → Existing Projects into Workspace**
2. `MMH_HAND.ioc`를 STM32CubeMX(또는 CubeIDE 내장 `.ioc` 에디터)로 열면 `Drivers/`(HAL/CMSIS)가 자동 재생성됩니다 — 저장소에는 커밋되어 있지 않습니다(§7)
3. `Project → Build Project` (Debug/Release)
4. 보드 연결 후 `Run → Debug`로 플래시 + Live Watch에서 `system_state` 값을 바꿔가며 동작 확인

---

## 7. GitHub 업로드 기준

**업로드 대상** (재현에 필수)
- `Core/Inc/*.h`, `Core/Src/*.c` (커스텀 로직 전체)
- `MMH_HAND.ioc`
- `STM32G474VETX_FLASH.ld`, `STM32G474VETX_RAM.ld`

**제외 대상** (`.gitignore`)

| 항목 | 이유 |
|---|---|
| `Drivers/` (`STM32G4xx_HAL_Driver`, `CMSIS`) | ST 표준 SDK. `.ioc` 기반으로 CubeMX가 자동 재생성 |
| `Debug/`, `Release/` | 빌드 산출물(`.elf`, `.map`, `.list` 등). 배포용 바이너리가 필요하면 `.elf`/`.bin`만 별도 릴리즈로 |
| `.settings/`, `.cproject`, `.project`, `.mxproject` | IDE 개인 워크스페이스 로컬 메타데이터 |

---

## 8. 알려진 제약 사항 / TODO

- [ ] `system_state == 2, 3, 5`가 아직 이식되지 않음 — `main.c`에 해당 분기가 없어 현재는 `0`(자유) / `1`(홈) / `4`(그립 시퀀스)만 동작함
- [ ] 통신 프로토콜 설계 (Live Watch → 실사용 방식으로 전환 필요, 방식 미정)
- [ ] `control_mode` 3번의 명칭 재검토 여부
- [ ] 힘 제어(force control) 로직에 `raw_current_adc` 통합 설계 — 현재는 `hand_sensor.c`에서 수집만 하고 `hand_motor.c` 로직에는 미사용
- [ ] `.gitignore` 작성 및 적용
