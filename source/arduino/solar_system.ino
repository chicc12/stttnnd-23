#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// Define servo objects
Servo horizontalServo;
Servo verticalServo;

// Initial servo positions
int horizontalPos = 110;
int verticalPos = 45;

// Servo movement limits
const int horizontalLimitHigh = 170;
const int horizontalLimitLow = 5;
const int verticalLimitHigh = 110;
const int verticalLimitLow = 25;

// LDR pin definitions
const int ldrTopLeft = A6;
const int ldrTopRight = A7;
const int ldrBottomLeft = A5;
const int ldrBottomRight = A4;

// Voltage sensor pin
const int voltageSensorPin = A0;
const float voltageFactor = 5.128;
const float arduinoVoltage = 5;

// Temperature sensor pin
const int oneWireBus = 6;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// AC Voltage sensor pin
const int acVoltageSensorPin = A1;

// Relay pins
const int relayPinA2 = A2;
const int relayPinA3 = A3;

// Switch pins
const int switchPin12 = 12;
const int switchPin13 = 13;
const int resetPin = 2; 
// LCD settings
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Navigation control variable
bool navigationEnabled = true;

// Error state variables
bool overheatError = false;
bool lowVoltageError = false;

double sensorValue=0;
double sensorValue1=0;
int crosscount=0;
int climbhill=0;
double VmaxD=0;
double VeffD;
double Veff;
// Function to read AC voltage
float readACVoltage(int pin) {
  sensorValue1=sensorValue;
delay(50);
sensorValue = analogRead(A1);
if (sensorValue>sensorValue1 && sensorValue>511){
  climbhill=1;
  VmaxD=sensorValue;
  }
if (sensorValue<sensorValue1 && climbhill==1){
  climbhill=0;
  VmaxD=sensorValue1;
  VeffD=VmaxD/sqrt(2);
  Veff=(((VeffD-420.76)/-90.24)*-210.2)+210.2;
  Serial.println(Veff);
  VmaxD=0;
}
  return Veff; // Adjusted factor for correct reading
}

// Function to navigate servos
void navigateServos(int topLeft, int topRight, int bottomLeft, int bottomRight) {
  int avgTop = (topLeft + topRight) / 2;
  int avgBottom = (bottomLeft + bottomRight) / 2;
  int avgLeft = (topLeft + bottomLeft) / 2;
  int avgRight = (topRight + bottomRight) / 2;
  int diffVertical = avgTop - avgBottom;
  int diffHorizontal = avgLeft - avgRight;
  const int tolerance = 50;
  if (abs(diffVertical) > tolerance) {
    if (avgTop > avgBottom) {
      verticalPos = min(verticalPos + 1, verticalLimitHigh);
    } else {
      verticalPos = max(verticalPos - 1, verticalLimitLow);
    }
    verticalServo.write(verticalPos);
    delay(50);
  }
  if (abs(diffHorizontal) > tolerance) {
    if (avgLeft > avgRight) {
      horizontalPos = max(horizontalPos - 1, horizontalLimitLow);
    } else {
      horizontalPos = min(horizontalPos + 1, horizontalLimitHigh);
    }
    horizontalServo.write(horizontalPos);
    delay(50);
  }
}
void resetESP8266() {
  Serial.println("Resetting ESP8266...");
  digitalWrite(resetPin, LOW);  // Set the reset pin to LOW
  delay(100);  // Wait for 100 milliseconds
  digitalWrite(resetPin, HIGH); // Set the reset pin to HIGH
  Serial.println("ESP8266 Reset done");
}
void setup() {
  Serial3.begin(112500);
  Serial.begin(9600); // Initialize the serial communication with the computer
   // Initialize the serial communication with the ESP8266
  horizontalServo.attach(8);
  verticalServo.attach(7);
  horizontalServo.write(horizontalPos);
  verticalServo.write(verticalPos);
  delay(3000);
  sensors.begin();
  lcd.init();
  lcd.backlight();
  pinMode(relayPinA2, OUTPUT);
  pinMode(relayPinA3, OUTPUT);
  pinMode(switchPin12, INPUT);
  pinMode(switchPin13, INPUT);
  pinMode(resetPin, OUTPUT);
 resetESP8266();
}

void loop() {
  float voltageSensorVal = analogRead(voltageSensorPin);
  float vOut = (voltageSensorVal / 1024.0) * arduinoVoltage;
  float vIn = vOut * voltageFactor;
  
  Serial.println(vIn);
  int topLeft = analogRead(ldrTopLeft);
  int topRight = analogRead(ldrTopRight);
  int bottomLeft = analogRead(ldrBottomLeft);
  int bottomRight = analogRead(ldrBottomRight);
  Serial.print(topLeft); Serial.print(" ");
  Serial.print(topRight); Serial.print(" ");
  Serial.print(bottomLeft); Serial.print(" ");
  Serial.print(bottomRight); Serial.println();
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print("Temp: "); Serial.print(temperatureC); Serial.println(" C");
  float acVoltage = readACVoltage(acVoltageSensorPin);
  Serial.print("AC Volt: "); Serial.print(acVoltage); Serial.println(" V");

  // Check error states
  overheatError = temperatureC > 50;
  lowVoltageError = vIn < 10;

  // Send data to ESP8266
   StaticJsonDocument<200> doc;
   if(navigationEnabled==true)doc["NAV"] = 1;
   else if(navigationEnabled==false)doc["NAV"] = 0;
  doc["IN"] = digitalRead(relayPinA2);
  doc["OUT"] = digitalRead(relayPinA3);
  doc["TEMP"] = temperatureC;
  doc["DC"] = vIn;
  doc["AC"] = acVoltage;
  if(overheatError==true)doc["OVERHEAT"] = 1;
  else if(overheatError==false)doc["OVERHEAT"] = 0;

  if(lowVoltageError==true)doc["LOW_VOLT"] = 1;
  else if(lowVoltageError==true)doc["LOW_VOLT"] = 0;

  // Serialize JSON to string
  String output;
  serializeJson(doc, output);

  Serial3.println(output);
  Serial.println(output);
  // Update display and control based on error states
  if (overheatError) {
    digitalWrite(relayPinA2, LOW);
    digitalWrite(relayPinA3, LOW);
    navigationEnabled = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Overheat Err");
    lcd.setCursor(0, 1);
    lcd.print("Shutdown");
  } else if (lowVoltageError) {
    digitalWrite(relayPinA3, LOW);
    navigationEnabled = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Low Volt Err");
    lcd.setCursor(0, 1);
    lcd.print("Shutdown");
  } else {
    digitalWrite(relayPinA2, HIGH);
    digitalWrite(relayPinA3, HIGH);
    navigationEnabled = true;
    if (navigationEnabled) {
      navigateServos(topLeft, topRight, bottomLeft, bottomRight);
    }
    int switch12 = digitalRead(switchPin12);
    int switch13 = digitalRead(switchPin13);
    lcd.clear();
    char buffer[6];
    if (switch12 == HIGH && switch13 == LOW) {
      lcd.setCursor(0, 0);
      lcd.print("Nav:");
      lcd.print(navigationEnabled ? "ON " : "OFF");
      lcd.setCursor(0, 1);
      lcd.print("OUT:");
      lcd.print(digitalRead(relayPinA3) ? "ON " : "OFF");
      lcd.print(" IN:");
      lcd.print(digitalRead(relayPinA2) ? "ON " : "OFF");
    } else if (switch12 == LOW && switch13 == HIGH) {
      lcd.setCursor(0, 0);
      lcd.print("T:");
      dtostrf(temperatureC, 4, 1, buffer);
      lcd.print(buffer);
      lcd.setCursor(0, 1);
      lcd.print("DC:");
      dtostrf(vIn, 4, 1, buffer);
      lcd.print(buffer);
      lcd.print(" AC:");
      dtostrf(acVoltage, 4, 1, buffer);
      lcd.print(buffer);
    } else if (switch12 == LOW && switch13 == LOW) {
      static unsigned long lastSwitchTime = 0;
      static int displayMode = 0;
      if (millis() - lastSwitchTime > 2000) {
        displayMode = (displayMode + 1) % 2;
        lastSwitchTime = millis();
      }
      if (displayMode == 0) {
        lcd.setCursor(0, 0);
        lcd.print("Nav:");
        lcd.print(navigationEnabled ? "ON " : "OFF");
        lcd.setCursor(0, 1);
        lcd.print("OUT:");
        lcd.print(digitalRead(relayPinA3) ? "ON " : "OFF");
        lcd.print(" IN:");
        lcd.print(digitalRead(relayPinA2) ? "ON " : "OFF");
      } else {
        lcd.setCursor(0, 0);
        lcd.print("T:");
        dtostrf(temperatureC, 4, 1, buffer);
        lcd.print(buffer);
        lcd.setCursor(0, 1);
        lcd.print("DC:");
        dtostrf(vIn, 4, 1, buffer);
        lcd.print(buffer);
        lcd.print(" AC:");
        dtostrf(acVoltage, 4, 1, buffer);
        lcd.print(buffer);
      }
    }
  }
  delay(100);
}
