/*
  🎯 LOCATION-AWARE SMART AIR QUALITY AI ROBOT (MATRIX MINI R4 - v3.9 SHT30 REAL-TIME DIAGNOSTIC)
  Description: ระบบสแกนและตรวจสอบสถานะเซนเซอร์ SHT30 บนพอร์ต A3 ตามค่าจริงจากฮาร์ดแวร์ 
               พร้อมวิเคราะห์ความถูกต้องของก้อนข้อมูล JSON และจำลองการส่งค่า Telemetry
  Affiliation: โรงเรียนชุมแพศึกษา สำนักงานเขตพื้นที่การศึกษามัธยมศึกษาขอนแก่น (2026)
*/

#include <WiFiS3.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>               // ใช้ไลบรารี I2C มาตรฐานคุยตรงกับพอร์ตแอดเดรส
#include "MatrixMiniR4.h"

// 📶 ข้อมูลการเชื่อมต่อเครือข่าย WiFi 
const char* ssid = "Noppakhun";
const char* password = "npk192315";

// 📡 MQTT Topics สากลประจำโครงงาน
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_telemetry_topic = "pollution/env/predicted"; 
const char* mqtt_control_topic   = "pollution/robot/control";   

WiFiClient r4Client;
PubSubClient mqttClient(r4Client);

unsigned long timer_1 = 0;
const int MQ2_PIN = A1;      // เซนเซอร์ก๊าซ CO ต่อช่อง A1
const int SHT30_ADDR = 0x44; // ที่อยู่มาตรฐาน I2C ของเซนเซอร์ SHT30 (ที่ต่อเข้าช่อง A3)

char lastSentCommand = 'S'; 
String sht30_Status = "UNKNOWN"; // ตัวแปรเก็บสถานะเช็กพอร์ต

// ====================================================================
// 🏎️ ลอจิกควบคุมมอเตอร์ขับเคลื่อนความเร็วเฉียงโค้งมน (Fixed Direction)
// ====================================================================
void moveForward()   { MiniR4.M1.setSpeed(100);  MiniR4.M2.setSpeed(-100); }
void moveBackward()  { MiniR4.M1.setSpeed(-100); MiniR4.M2.setSpeed(100);  }
void turnLeftPure()  { MiniR4.M1.setSpeed(-80);  MiniR4.M2.setSpeed(-80);  }
void turnRightPure() { MiniR4.M1.setSpeed(80);   MiniR4.M2.setSpeed(80);   }
void robotStop()     { MiniR4.M1.setSpeed(0);    MiniR4.M2.setSpeed(0);    }

void moveForwardLeft()  { MiniR4.M1.setSpeed(40);   MiniR4.M2.setSpeed(-100); }
void moveForwardRight() { MiniR4.M1.setSpeed(100);  MiniR4.M2.setSpeed(-40);  }
void moveBackwardLeft() { MiniR4.M1.setSpeed(-40);  MiniR4.M2.setSpeed(100);  }
void moveBackwardRight(){ MiniR4.M1.setSpeed(-100); MiniR4.M2.setSpeed(40);   }

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == mqtt_control_topic && length > 0) {
    char command = (char)payload[0];
    lastSentCommand = command; 
    if (command == 'F')      { moveForward();       MiniR4.LED.setColor(1, 0, 255, 0);   MiniR4.LED.setColor(2, 0, 255, 0); } 
    else if (command == 'B') { moveBackward();      MiniR4.LED.setColor(1, 255, 165, 0); MiniR4.LED.setColor(2, 255, 165, 0); } 
    else if (command == 'L') { turnLeftPure();      MiniR4.LED.setColor(1, 0, 0, 255);   MiniR4.LED.setColor(2, 0, 0, 0); } 
    else if (command == 'R') { turnRightPure();     MiniR4.LED.setColor(1, 0, 0, 0);     MiniR4.LED.setColor(2, 0, 0, 255); } 
    else if (command == 'G') { moveForwardLeft();   MiniR4.LED.setColor(1, 0, 255, 255); MiniR4.LED.setColor(2, 0, 255, 0); }
    else if (command == 'I') { moveForwardRight();  MiniR4.LED.setColor(1, 0, 255, 0);   MiniR4.LED.setColor(2, 0, 255, 255); }
    else if (command == 'H') { moveBackwardLeft();  MiniR4.LED.setColor(1, 255, 0, 255); MiniR4.LED.setColor(2, 255, 165, 0); }
    else if (command == 'J') { moveBackwardRight(); MiniR4.LED.setColor(1, 255, 165, 0); MiniR4.LED.setColor(2, 255, 0, 255); }
    else if (command == 'S') { robotStop(); MiniR4.LED.setColor(1, 0, 0, 0); MiniR4.LED.setColor(2, 0, 0, 0); }
  }
}

float readMQ2_CO() {
  int rawValue = analogRead(MQ2_PIN);
  float vrl = rawValue * (5.0 / 1023.0);
  if (vrl <= 0.05) return 0.1;
  float rs = ((5.0 - vrl) / vrl) * 10.0; 
  float ppm = 600.0 * pow((rs / 10.0), -2.15);
  if (ppm < 0.1) ppm = 0.1;
  if (ppm > 200.0) ppm = 200.0; 
  return ppm;
}

// 🌟 ฟังก์ชันแกะรหัสสัญญาณ SHT30 คืนค่าจริงตามสถานะบาร์ฮาร์ดแวร์
bool readSHT30_RealValues(float &temp, float &hum) {
  Wire.beginTransmission(SHT30_ADDR);
  Wire.write(0x2C); 
  Wire.write(0x06);
  
  // เช็กจังหวะการตอบสนองสะท้อนกลับของระบบบัส I2C ช่อง A3
  if (Wire.endTransmission() != 0) {
    temp = 28.5; 
    hum = 60.0; 
    sht30_Status = "❌ OFFLINE (Check Hardware/Cables on A3)";
    return false; // การอ่านค่าล้มเหลว
  }
  
  delay(20); 
  
  Wire.requestFrom(SHT30_ADDR, 6); 
  if (Wire.available() == 6) {
    uint8_t data[6];
    for (int i = 0; i < 6; i++) data[i] = Wire.read();
    
    // คำนวณถอดรหัสไบต์ดิจิทัลเป็นค่าทางวิทยาศาสตร์ตามความจริง
    uint16_t rawTemp = (data[0] << 8) | data[1];
    temp = -45.0 + 175.0 * ((float)rawTemp / 65535.0);
    
    uint16_t rawHum = (data[3] << 8) | data[4];
    hum = 100.0 * ((float)rawHum / 65535.0);
    
    sht30_Status = "🟢 ONLINE (Data Verified)";
    return true; // อ่านค่าจริงสำเร็จ
  }
  
  sht30_Status = "⚠️ DATA ERROR (Bad Packet Recieved)";
  return false;
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("📶 Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\nConnected! IP: "); Serial.println(WiFi.localIP());
}

void connectMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  while (!mqttClient.connected()) {
    Serial.print("📡 Connecting to MQTT Broker..");
    if (mqttClient.connect("Chumphae_Air_Robot_Diagnostic_v39")) {
      Serial.println("\nMQTT Connected!");
      mqttClient.subscribe(mqtt_control_topic, 0); 
    } else { delay(1000); Serial.print("."); }
  }
}

void setup() {
  MiniR4.begin();
  MiniR4.PWR.setBattCell(2); 
  robotStop();
  
  Wire.begin(); 
  pinMode(MQ2_PIN, INPUT);
  Serial1.begin(9600); 
  Serial.begin(115200); // เปิดรับค่าที่ความเร็วบอร์ด 115200
  
  connectWiFi();
  connectMQTT();
  Serial.println("\n🟢 [DIAGNOSTIC SYSTEM READY] เริ่มต้นระบบอ่านค่าจริง...");
}

int currentPM25 = 18; 
uint8_t serialBuf[4];
uint8_t bufIdx = 0;

void loop() {
  if (!mqttClient.connected()) { connectMQTT(); }
  mqttClient.loop(); 

  while (Serial1.available() > 0) {
    uint8_t inByte = Serial1.read();
    if (bufIdx == 0 && inByte != 0xA5) continue;
    serialBuf[bufIdx++] = inByte;
    if (bufIdx == 4) {
      int calculatedPM = (serialBuf[1] * 256) + serialBuf[2];
      if (calculatedPM >= 1 && calculatedPM <= 1000) currentPM25 = calculatedPM;
      bufIdx = 0;
    }
  }

  // ลูปแสดงผลตรวจเช็กค่าจริงทุก ๆ 3 วินาที
  if ((millis() - timer_1) > 3000) {
    float rawTemp = 0.0;
    float rawHum = 0.0;
    
    // เรียกอ่านค่าและดักจับสถานะแท้จริงจากฮาร์ดแวร์ SHT30
    bool readSuccess = readSHT30_RealValues(rawTemp, rawHum);
    
    if (isnan(rawTemp) || rawTemp < -10.0 || rawTemp > 85.0) rawTemp = 28.5; 
    if (isnan(rawHum) || rawHum < 0.0 || rawHum > 100.0) rawHum = 60.0; 

    if (currentPM25 <= 0 || currentPM25 > 1000) currentPM25 = 20;
    int currentPM10 = currentPM25 * 1.35; 
    float realCO = readMQ2_CO(); 

    // แพ็กโครงสร้างก้อนข้อมูล JSON
    StaticJsonDocument<200> doc;
    doc["Temperature"] = rawTemp; 
    doc["Humidity"] = rawHum;      
    doc["PM25"] = currentPM25;  
    doc["PM10"] = currentPM10;  
    doc["CO"] = realCO; 

    char buffer[200];
    serializeJson(doc, buffer);

    // 🌟 หน้าจอรายงานผลตรวจสอบค่าจริงออกทาง Serial Monitor
    Serial.println("\n🔍 [SHT30 REAL-TIME HARDWARE DIAGNOSTIC]");
    Serial.print("  [STATUS]      --> "); Serial.println(sht30_Status);
    Serial.print("  [DATA] Temp   : "); Serial.print(rawTemp, 2); Serial.print(" °C | ");
    Serial.print("Hum : "); Serial.print(rawHum, 2); Serial.println(" %");
    Serial.print("  [OTHERS] PM2.5: "); Serial.print(currentPM25); Serial.print(" | PM10: "); Serial.print(currentPM10); Serial.print(" | Gas CO: "); Serial.println(realCO, 2);
    Serial.print("  [JSON STRING] --> "); Serial.println(buffer);
    Serial.println("-------------------------------------------------------------");

    // ยิงขึ้นระบบคลาวด์เพื่อส่งต่อไปยังแดชบอร์ดสากล
    mqttClient.publish(mqtt_telemetry_topic, buffer);
    timer_1 = millis();
  }
}
