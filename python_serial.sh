python3 - <<'EOF'
import serial
s = serial.Serial()
s.port = '/dev/ttyACM0'; s.baudrate = 115200
s.dtr = False; s.rts = False
s.open()
while True:
    print(s.readline().decode(errors='replace'), end='')
EOF
