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
