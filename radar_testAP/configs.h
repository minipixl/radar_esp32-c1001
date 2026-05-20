#define mqttserverIP "192.168.1.139"

// WROVER KIT entries device1
// #define I2C_SDA 21
// #define I2C_SCL 22
// offsets seit 04.01.2025
// #define temp_offset -6
// #define hum_offset 8
// #define RAUM "home/room1/air";
// #define STATS "home/room1/status";
// #define DEVICE "esp32-room1"

// WROVER KIT entries device2
// #define I2C_SDA 6
// #define I2C_SCL 7
// offsets seit 04.01.2025
// #define temp_offset -6.5
// #define hum_offset 7
// #define RAUM "home/room2/air";
// #define STATS "home/room2/status";
// #define DEVICE "esp32-room2"

// ESP32-C6 SuperMini + GY-BME280 (Adafruit Bibliothek, nur Temp/Hum/Pres/Alt)
#define I2C_SDA       6
#define I2C_SCL       7
#define BME280_ADDR   0x76    // SDO an GND = 0x76 / SDO an 3.3V = 0x77
// offsets (nach Kalibrierung anpassen)
#define temp_offset   0
#define hum_offset    0
#define RAUM "home/balc1/air";
#define STATS "home/balc1/status";
#define DEVICE "esp32-balc1"