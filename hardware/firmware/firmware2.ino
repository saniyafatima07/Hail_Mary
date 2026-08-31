#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// --- Pin Definitions ---
#define MASTER_BUTTON_PIN 25  // Button 1: Changes Modes
#define NAV_BUTTON_PIN 26     // Button 2: Changes Nav Screens
#define VIB_PIN 32
#define IR_PIN 33
const int MPU_ADDR = 0x68;

// --- Objects ---
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_BMP280 bmp;
Adafruit_INA219 ina219;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// --- Master State Variables ---
int masterMode = 0;             
unsigned long lastMasterPress = 0;
unsigned long lastNavPress = 0;
unsigned long lastScreenUpdate = 0;

// Nav Variables (Mode 0)
int navScreen = 0;              

// Timer Variables (Mode 3)
unsigned long timerStartTime = 0;

// Impact Variables (Mode 2)
unsigned long impactStartTime = 0;
bool impactAlertActive = false;

// --- DIGITAL CAPACITOR VARIABLES ---
int virtualCapacitor = 0;
const int CAP_MAX = 1000;    
const int THRESHOLD = 800;   

void setup() {
  Serial.begin(115200);
  
  pinMode(MASTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(NAV_BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT);
  pinMode(VIB_PIN, INPUT);
  
  Wire.begin(21, 22);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED Failed"); while(1);
  }
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,20); display.println("Booting Master..."); display.display();

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

  ina219.begin(&Wire);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  delay(1000);
}

void loop() {
  // --- 1. Background Tasks (Runs thousands of times per second) ---
  
  // A. Keep GPS alive
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read()); 
  }

  // B. DIGITAL CAPACITOR FOR IMPACT SENSOR
  if (masterMode == 2) {
    if (digitalRead(VIB_PIN) == HIGH) { // Change to LOW if Active-Low
      virtualCapacitor += 10; 
      if (virtualCapacitor > CAP_MAX) virtualCapacitor = CAP_MAX; 
    } else {
      virtualCapacitor -= 2;  
      if (virtualCapacitor < 0) virtualCapacitor = 0;             
    }

    if (virtualCapacitor > THRESHOLD && !impactAlertActive) { 
      impactAlertActive = true;
      impactStartTime = millis();
      virtualCapacitor = 0; 
    }
  }

  // --- 2. Button Logic (Instant Response) ---
  
  // Master Button (Pin 25)
  if (digitalRead(MASTER_BUTTON_PIN) == LOW) {
    if (millis() - lastMasterPress > 300) { 
      masterMode++;
      if (masterMode > 3) masterMode = 0;
      
      if (masterMode == 3) {
        timerStartTime = millis();
      }
      
      lastMasterPress = millis();
      lastScreenUpdate = 0; // Force instant refresh
    }
  }

  // Nav Button (Pin 26) - Only does something if we are in Mode 0!
  if (digitalRead(NAV_BUTTON_PIN) == LOW && masterMode == 0) {
    if (millis() - lastNavPress > 300) { 
      navScreen++;
      if (navScreen > 3) navScreen = 0;
      
      lastNavPress = millis();
      lastScreenUpdate = 0; // Force instant screen update
    }
  }


  // --- 3. Screen Drawing ---
  if (millis() - lastScreenUpdate >= 100) {
    lastScreenUpdate = millis();
    display.clearDisplay();
    display.setCursor(0, 0);

    // ==========================================
    // MODE 0: NAVIGATION (Manual Button Control)
    // ==========================================
    if (masterMode == 0) {
      display.setTextSize(1);
      
      if (navScreen == 0) { 
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 6, true);
        int16_t AcX = Wire.read()<<8 | Wire.read(); int16_t AcY = Wire.read()<<8 | Wire.read(); int16_t AcZ = Wire.read()<<8 | Wire.read();
        display.println("--- [0] NAV: MPU ---"); display.println("");
        display.print("X: "); display.println(AcX); display.print("Y: "); display.println(AcY); display.print("Z: "); display.println(AcZ);
      } 
      else if (navScreen == 1) { 
        display.println("--- [0] NAV: BMP ---"); display.println("");
        display.print("Temp: "); display.print(bmp.readTemperature()); display.println(" C");
        display.print("Pres: "); display.print(bmp.readPressure() / 100.0F); display.println(" hPa");
        display.print("Alt:  "); display.print(bmp.readAltitude(1013.25)); display.println(" m");
      } 
      else if (navScreen == 2) { 
        display.println("--- [0] NAV: PWR ---"); display.println("");
        display.print("Volt: "); display.print(ina219.getBusVoltage_V()); display.println(" V");
        display.print("Curr: "); display.print(ina219.getCurrent_mA()); display.println(" mA");
        display.print("Pwr:  "); display.print(ina219.getPower_mW()); display.println(" mW");
      } 
      else if (navScreen == 3) { 
        display.println("--- [0] NAV: GPS ---"); display.println("");
        display.print("Sats: "); display.println(gps.satellites.value());
        if (gps.location.isValid()) {
          display.print("Lat: "); display.println(gps.location.lat(), 5); display.print("Lon: "); display.println(gps.location.lng(), 5);
        } else {
          display.println("Searching...");
        }
      }
    }

    // ==========================================
    // MODE 1: PROXIMITY (IR Sensor)
    // ==========================================
    else if (masterMode == 1) {
      display.setTextSize(1); display.println("--- [1] PROXIMITY ---"); display.println("");
      if (digitalRead(IR_PIN) == LOW) { 
        display.setTextSize(2); display.println("OBJECT!"); display.println("DETECTED");
      } else {
        display.setTextSize(2); display.println("CLEAR"); display.setTextSize(1); display.setCursor(0, 45); display.println("Path is open.");
      }
    }

    // ==========================================
    // MODE 2: IMPACT (Digital Capacitor)
    // ==========================================
    else if (masterMode == 2) {
      if (impactAlertActive == true && (millis() - impactStartTime >= 3000)) {
        impactAlertActive = false;
      }

      display.setTextSize(1); display.println("--- [2] IMPACT ---"); display.println("");
      
      if (impactAlertActive == true) {
        display.setTextSize(2); 
        display.println("IMPACT!");
        
        display.setTextSize(1);
        display.setCursor(0, 50);
        display.print("Clearing in: ");
        display.print(3 - ((millis() - impactStartTime) / 1000));
        display.println("s");
      } else {
        display.setTextSize(2);
        display.println("SAFE");
        display.setTextSize(1);
        display.setCursor(0, 45);
        display.println("No vibration.");
      }
    }

    // ==========================================
    // MODE 3: COUNTDOWN TIMER (5 seconds)
    // ==========================================
    else if (masterMode == 3) {
      display.setTextSize(1); display.println("--- [3] TIMER ---"); display.println("");
      
      long remainingTime = 5000 - (millis() - timerStartTime);
      
      if (remainingTime <= 0) {
        display.setTextSize(2); 
        display.println("00:00.00");
        display.setTextSize(1); display.println(""); display.println("  DONE!");
      } else {
        int secs = (remainingTime / 1000);
        int ms = (remainingTime % 1000) / 10;

        display.setTextSize(2);
        display.print("00:0"); display.print(secs);
        display.print(".");
        if (ms < 10) display.print("0"); 
        display.println(ms);
      }
    }
    
    display.display();
  }
}