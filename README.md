# esp32-c1001
Human presence detection with esp32 and dfrobot c1001. \
\
Sensor detects:
presence:  nobody, somebody, error\
movement status:  no movement, still, active\
movementParam: value 0..1000\
respirationRate\
heartRate\
\
Two startup modi:\
GP8 on VCC:  start as access point\
GP8 on GND or open: start normal WIFI mode\
<img width="588" height="775" alt="Bildschirmfoto vom 2026-07-26 14-50-24" src="https://github.com/user-attachments/assets/d76c137d-18df-4482-8eca-d597674f7324" />

Wiring scheme C1001 with esp32-c6 supermini:\
VIN - 5V \
GND - GND \
TX - GP7 \
RX - GP6 \
<img width="794" height="1123" alt="wiring" src="https://github.com/user-attachments/assets/5acaae30-a4ae-49d3-b7ff-bbf1bdb35525" />






