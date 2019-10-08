#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <QuickStats.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define VREF 5.0

// Stat. computation
#define SAMPLES 10
QuickStats stats;
unsigned int sampleIndex;

// LCD
#define OLED_ADDRESS 0x3C
#define OLED_RESET_PIN -1
SSD1306AsciiWire oled;

// TDS
#define TDS_PIN A1
float tdsValue, tdsValues[SAMPLES];

// Temperature
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempAddress;
unsigned long tempTime;
int tempResolution = 10;
int tempDelay = 750 / (1 << (12 - tempResolution));
float tempValue = 25;

// Pressure
#define PRESS_ALARM 800
#define PRESS_VMIN 0.88 // 0.004A x 220R = 0.88V
#define PRESS_VMAX 4.4 // 0.02A × 220R = 4.4V
#define PRESS_SENSOR_MAX 1450.38
#define PRESS_PIN A2
float pressValue, pressValues[SAMPLES];

// Flow meters
// In
#define FLOW_0_INTERRUPT 0 // 0 = digital pin 2
#define FLOW_0_PIN 2
#define FLOW_0_Q 4.5 // f=4.5*Q; Q=L/min; f=hZ
// Out
#define FLOW_1_INTERRUPT 1 // 1 = digital pin 3
#define FLOW_1_PIN 3
#define FLOW_1_Q 55 // f=55*q
#define FLOW_FREQ 0.5 // Seconds per update
unsigned long flowTime;
unsigned int flowPulses0, flowPulses1;  
float flowRate0, flowRate1;

// Interrupt functions for updating pulse counts
void pulseFlow0() { flowPulses0++; }
void pulseFlow1() { flowPulses1++; }

// Show data on LCD screen
void updateScreen() {
  char buffer[16];
  oled.setCursor(0, 0);

  // Inflow
  sprintf(buffer, "In:%4d.%02d l/min", (int)flowRate0, (int)(flowRate0*100)%100);
  oled.println(buffer);

  // Outflow
  sprintf(buffer, "Out:%3d.%02d l/min", (int)flowRate1, (int)(flowRate1*100)%100);
  oled.println(buffer);

  // TDS
  if (tempValue < 0) sprintf(buffer, "TDS:*TEMP ERROR*");
  else if (tdsValue <= 0) sprintf(buffer, "TDS:     *ERROR*");
  else sprintf(buffer, "TDS:%8d ppm", (int)tdsValue);
  oled.println(buffer);

  // Pressure
  // if (pressValue < 0) sprintf(buffer, "Press:   *ERROR*"); // Beware analogue drift
  if (false) return; // I got 4-20ma not TTL by accident
  else sprintf(buffer, "Press:%6d PSI", (int)pressValue); // Can't visualise non-ogre p unit
  oled.println(buffer);

  // Pressure alarm
  if (pressValue > PRESS_ALARM) {
    oled.invertDisplay(true);
    oled.setInvertMode(true);
    oled.setCursor(1.5*8, 3);
    oled.println(" PRES. ALARM ");
    oled.setInvertMode(false);
    delay(100);
    oled.invertDisplay(false);
    return;
  }
}

void updateFlow() {
  // Measure
  float elapsed = (millis() - flowTime) / 1000;
  if (elapsed < FLOW_FREQ) return;
  flowRate0 = (1 / elapsed * flowPulses0) / FLOW_0_Q;
  flowRate1 = (1 / elapsed * flowPulses1) / FLOW_1_Q;
  // Reset
  flowPulses0 = 0;
  flowPulses1 = 0;
  flowTime = millis();
}

void updateTds() {
  float voltage = (float)analogRead(TDS_PIN) * VREF / 1023.0;
  float compensationCoefficient = 1.0 + 0.02 * max(tempValue - 25.0, 0);
  float compensationVoltage = voltage / compensationCoefficient;
  tdsValues[sampleIndex] = (
    133.42 * pow(compensationVoltage, 3)
    - 255.86 * pow(compensationVoltage, 2)
    + 857.39 * compensationVoltage
  ) / 2; // Presenting: some nice formula's severed torso
  tdsValue = stats.median(tdsValues, SAMPLES);
}

void updatePressure() {
  float voltage = (float)analogRead(PRESS_PIN) * VREF / 1023.0; // I bet there's a stdlib somewhere
  pressValues[sampleIndex] = PRESS_SENSOR_MAX
   * (voltage - PRESS_VMIN)
   / (PRESS_VMAX - PRESS_VMIN);
  pressValue = stats.median(pressValues, SAMPLES);
}

void updateTemperature() {
  if (millis() - tempTime < tempDelay) return;
  if (tempTime != 0) tempValue = sensors.getTempCByIndex(0);
  sensors.requestTemperatures();
  tempTime = millis ();
}

void setup() {
  // Setup LCD
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, OLED_ADDRESS, OLED_RESET_PIN);
  oled.setFont(ZevvPeep8x16);
  oled.clear();

  // Setup flow meters
  flowTime = millis();
  pinMode(FLOW_0_PIN, INPUT);
  pinMode(FLOW_1_PIN, INPUT);
  digitalWrite(FLOW_0_PIN, HIGH);
  digitalWrite(FLOW_1_PIN, HIGH);
  attachInterrupt(FLOW_0_INTERRUPT, pulseFlow0, FALLING);
  attachInterrupt(FLOW_1_INTERRUPT, pulseFlow1, FALLING);

  // Setup temp
  sensors.begin();
  sensors.getAddress(tempAddress, 0);
  sensors.setResolution(tempAddress, tempResolution);
  sensors.setWaitForConversion(false);

  // Setup TDS
  pinMode(TDS_PIN, INPUT);

  // Setup pressure
  pinMode(PRESS_PIN, INPUT);
}

void loop() {
  updateFlow();
  updateTemperature();
  updateTds();
  updatePressure();
  updateScreen();

  // Update sample counter
  sampleIndex++;
  if (sampleIndex == SAMPLES) sampleIndex = 0; // I don't do >= because I like living on the edge
}
