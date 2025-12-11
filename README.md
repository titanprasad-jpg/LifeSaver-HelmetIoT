
# 🚨 Smart Helmet for Accident Detection Using IoT

The **Smart Helmet** is an IoT-based safety system designed to detect accidents and automatically alert emergency contacts with the rider’s GPS location. It aims to reduce road fatalities by enabling faster rescue response and promoting helmet usage.

---

## 📌 Features
- **Accident Detection:** Uses MPU6050 accelerometer and gyroscope to measure sudden impact.
- **Automatic SMS Alert:** Sends emergency message with GPS coordinates via GSM (SIM800L).
- **Helmet Usage Detection:** Ignition interlock ensures bike starts only when helmet is worn.
- **Real-Time Location:** SMS includes a Google Maps link.
- **Low-Cost, Portable System:** Easily installable in any helmet.

---

## 🧩 Components Used
- ESP32 / NodeMCU microcontroller  
- MPU6050 accelerometer & gyroscope  
- GPS module (NEO-6M)  
- GSM module (SIM800L)  
- Helmet-wear detection switch  
- Rechargeable battery and wiring  

---

## 🔧 Working Methodology
1. Helmet is worn → switch activates.  
2. MPU6050 monitors acceleration and angular motion.  
3. If values exceed the accident threshold → crash detected.  
4. Microcontroller reads location from GPS module.  
5. GSM module sends SMS to emergency contact.  
6. Message includes **Google Maps location link**.  
7. Optional buzzer/app confirms transmission.

---

## 🧱 Block Diagram
Impact → MPU6050 → ESP32/NodeMCU → GSM → SMS
↓
GPS Module → Location


---

## ⭐ Advantages
- Faster emergency response  
- Reduces road accident fatalities  
- Works in rural & urban areas  
- Compatible with any helmet  
- Low cost and easy to build  

---

## 🎯 Applications
- Two-wheeler riders  
- Delivery/logistics personnel  
- Police & traffic departments  
- Smart-city safety systems  
- College transportation  

---

## 🚀 Future Scope
- Mobile app with live tracking  
- Crash video recording  
- Health monitoring (heart rate, temperature)  
- Automatic ambulance calling  
- Govt. transport safety enforcement  

---

## 📌 Conclusion
The Smart Helmet integrates IoT, sensors, and wireless communication to deliver a practical accident detection and alert system. Its simplicity, affordability, and effectiveness make it a valuable safety solution capable of saving lives.

---
## Full code

✅ FULL CODE

#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <MPU6050.h>

MPU6050 mpu;
TinyGPSPlus gps;

// Pins
#define GPS_RX 16
#define GPS_TX 17
#define GSM_RX 27
#define GSM_TX 26
#define IR_SENSOR 14
#define RELAY_PIN 25
#define BUZZER 12
#define LED_PIN 13

HardwareSerial gsm(1);
HardwareSerial gpsSerial(2);

// Thresholds
const float CRASH_THRESHOLD = 3.5;  // adjust for your helmet impact
const int TILT_ANGLE = 45;          // tilt detection for fall

void setup() {
  Serial.begin(115200);

  pinMode(IR_SENSOR, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW); // ignition off by default
  digitalWrite(BUZZER, LOW);
  digitalWrite(LED_PIN, LOW);

  // Start serial modules
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  gsm.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);

  // Init MPU6050
  Wire.begin();
  mpu.initialize();

  Serial.println("Smart Helmet System Started");
}

void loop() {

  // -------------------
  //  1. Helmet Detection
  // -------------------
  if (digitalRead(IR_SENSOR) == LOW) {
    digitalWrite(RELAY_PIN, HIGH); // allow ignition
  } else {
    digitalWrite(RELAY_PIN, LOW); // block ignition
  }

  // -------------------
  //  2. Accident Detection
  // -------------------
  mpu.getAcceleration();
  float ax = mpu.getAccelerationX() / 16384.0;
  float ay = mpu.getAccelerationY() / 16384.0;
  float az = mpu.getAccelerationZ() / 16384.0;

  float impact = sqrt(ax * ax + ay * ay + az * az);

  float tilt = abs(atan2(ay, az) * 57.2958);

  if (impact > CRASH_THRESHOLD && tilt > TILT_ANGLE) {
    Serial.println("ACCIDENT DETECTED");
    digitalWrite(BUZZER, HIGH);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);
    digitalWrite(BUZZER, LOW);

    sendEmergencySMS();
    delay(5000);
  }

  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

void sendEmergencySMS() {

  float lat = gps.location.lat();
  float lng = gps.location.lng();

  gsm.println("AT+CMGF=1");
  delay(500);
  gsm.println("AT+CMGS=\"+91XXXXXXXXXX\""); // your number here
  delay(500);

  gsm.print("ALERT! Accident detected.\n");
  gsm.print("Location: https://www.google.com/maps/?q=");
  gsm.print(lat, 6);
  gsm.print(",");
  gsm.print(lng, 6);

  delay(500);
  gsm.write(26); // End SMS
  Serial.println("SMS Sent!");
}
## ⭐ HOW THE WHOLE SYSTEM THINKS (AI Flow Logic)

Below is a clean AI-generated explanation of the internal workflow:

START

↓  
Check Helmet Sensor  
↓  
IF Helmet NOT Worn → Keep Ignition OFF  
IF Helmet Worn → Enable Ignition and Start Monitoring  

↓  
Read MPU6050 Data (Every 100 ms)  
↓  
IF Sudden Impact + High Tilt  
        → Trigger Accident Mode  
        → Get GPS Coordinates  
        → Send SMS via GSM  
        → Activate Buzzer / Alert LED  
ELSE  
        → Continue Monitoring
        
## 👤 Author
**peetha jaya durga prasad**  
Smart Helmet IoT Project

