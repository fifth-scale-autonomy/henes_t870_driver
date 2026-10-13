# HENES T870 아두이노 제어기 펌웨어 (참고용 사본)

출처: `~/Downloads/4300U_Win.zip` → `HENES/HENES_Platform_Arduino(최종)/Arduino/henes_lcd_pid/henes_lcd_pid.ino`
(예전 Windows + LabVIEW 로 이 차량을 조종하던 자료). 이 값들로 실차에서 속도·조향이 정상 동작함을 확인했다 (2026-10-08, 조향 명령 ±2000 → 피드백 약 ±1900).

scale_vehicle_node 가 이 펌웨어에 맞춘 부분 (2026-10-08):

| 항목 | 펌웨어 | 드라이버 |
|---|---|---|
| 속도 Hsp | 0~1000 을 받아 `map(Hsp,0,1000,0,15)` | `speed(0~1) * speed_max(1000)` |
| 조향 Hst | ±2000 = 조향 끝 (예전 주행 기록 약 ±20도) | `rad / radians(steer_full_lock_deg=20) * 2000` |
| 수신 | 루프마다 새 패킷이 없으면 그 루프는 Hsp=0, 중립 | 100 Hz 로 송신 (루프 약 10~25 ms) |
| 모드 Ham / 비상 Hes | Ham==1 이어야 PC 명령을 따름 | 1 / 0 |

피드백의 모드(AorM)·비상(Les)·기어(Lge)는 **조종기(RC) 스위치 상태**다. PC 명령이 아니다.
- 자율 주행: 조종기 모드 채널 펄스 < 1250 µs ("Auto Mode" 가 LCD 에 표시)
- 비상정지: 조종기 스로틀 펄스 < 1200 µs
