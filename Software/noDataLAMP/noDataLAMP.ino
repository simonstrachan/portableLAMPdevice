#include <PID_v1.h>             //PID library
#include <Wire.h>               //I2C library
#include "Adafruit_TCS34725.h"  //RGB color sensor library

//LAMP values
const double targetTemp = 82.5;
const float endTime = 30;

//Initialising Pins
#define fetPin 11
#define thermoPin A0

//Time Variables
uint32_t timer, timeMinutes, timeSeconds, elapsedTime, currentMillis, previousMillis, prev_timeSeconds, startTime;
uint16_t RGBinterval = 700;  //Time in milliseconds

//Temperature Variables
const uint32_t thermoOhm = 100000;       //NTC nominal resistance
const int8_t thermoOhmTemp = 25;         //NTC temperature for nominal resistance
const uint16_t thermoBeta = 3950;        //Beta for NTC
const uint32_t seriesResistor = 100000;  //Value of R1
const uint8_t numSamples = 250;
int16_t samples[numSamples];
double adcvalue, thermoValue, set, adj_set, steinhart;

//RGB -> XYZ matrices
const float patchMatrix[3][3] = { { 0.004471237, 0.003999354, -0.0001078278 }, { 0.000305326, 0.01103072, -0.0032857488 }, { -0.0022756, -0.00100645, 0.012742875 } };
float rgb[3], XYZ[3];
Adafruit_TCS34725 tcs[] = { Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X),
                            Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X),
                            Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X),
                            Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X) };
const uint8_t TCScount = 4;
#define TCAADDR 0x70

//Color Index Value variables
bool initialCIVfound[] = { false, false, false, false };
float initialCIV[TCScount];
float CIV, zeroCIV = 0;

// Moving average Variables
const uint8_t window_size = 15;
uint8_t index = 0;
float sum, averaged = 0;
float readings[window_size];

//Testing positive variables
const uint8_t numReadings = 20;
float pos_readings[TCScount][numReadings];
float currentReadings[TCScount];
int pos_count[] = { 0, 0, 0, 0 };
uint8_t currentIndex = 0;
bool positive = false;

//Other Variables
uint8_t i, inChar;
bool ready, running = false;

//PID initialisation
const double Kp = 48;
const double Ki = 1.352113;
const double Kd = 426;
PID myPID(&steinhart, &set, &targetTemp, Kp, Ki, Kd, DIRECT);

void setup(void) {
  Serial.begin(9600);
  Wire.begin();
  initTCS();

  analogReference(EXTERNAL);  //3V3 AREF

  myPID.SetMode(AUTOMATIC);
  for (int i = 3; i <= 10; i++) {
    pinMode(i, OUTPUT);
    if (i % 2 == 1) {
      digitalWrite(i, HIGH);
    }
  }
  pinMode(fetPin, OUTPUT);

  startTime = millis();
}

void loop(void) {
  if (Serial.available() > 0) {
    inChar = Serial.read();
  }
  //Press 's' to start heating and 'a' to start experiment
  if (!running && inChar == 'a') {
    running = true;
    startTime = millis();
    prev_timeSeconds = 0;
  } else if (!ready && inChar == 's') {
    ready = true;
    startTime = millis();
    prev_timeSeconds = 0;
  }

  if (ready) {
    //Read NTC and update temperature value
    steinhartCalc();
    //Use PID to set FET PWM value
    myPID.Compute();
    adj_set = set / 4;
    analogWrite(fetPin, adj_set);
  }



  if (running) {
    //Only get RGB data if the integration time has passed
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= RGBinterval) {
      for (int i = 0; i < TCScount; i++) {
        RGBsensor(i);
      }
      previousMillis = currentMillis;
    }
    for (int i = 0; i < TCScount; i++) {
      if (pos_count[i] >= 3) {
        int pin = 3 + i;
        digitalWrite(pin - 1, LOW);
        digitalWrite(pin, HIGH);
      }
    }
  }

  //Timer details
  printTimer();

  //Check if time is up
  if (timer >= (endTime * 60)) {
    endScreen();
  }
}

void initTCS() {
  for (int i = 0; i < TCScount; i++) {
    tcaselect(i);
    if (tcs[i].begin()) {
      Serial.println("Found sensor ");
    } else {
      Serial.println("No Sensor Found");
      while (true)
        ;
    }
  }
}

void tcaselect(uint8_t bus) {
  if (bus > TCScount) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << (bus + 4));
  Wire.endTransmission();
}

//Calculate NTC temperature
void steinhartCalc() {
  //Read NTC and average multiple samples for stability
  for (i = 0; i < numSamples; i++) {
    samples[i] = analogRead(thermoPin);
  }
  adcvalue = 0;

  //averaging samples
  for (i = 0; i < numSamples; i++) {
    adcvalue += samples[i];
  }
  adcvalue /= numSamples;

  /*            Math
     Voltage divider: Vo=Vcc(R2/R1+R2)
     ADC value = vo*1023/Varef
     ADC value = Vcc(R2/R1+R2)*1023/Varef
     Varef=Vcc
     ADC value= (R2/R1+R2)*1023
     R2 = - (R1 * ADC value)/(ADC value - 1023)
  */
  thermoValue = -(seriesResistor * adcvalue) / (adcvalue - 1023);

  /*   Steinhart-Hart Equation
     1/T = (1/To) + 1/B * ln(R/Ro)
  */
  steinhart = thermoValue / thermoOhm;          // (R/Ro)
  steinhart = log(steinhart);                   // ln(R/Ro)
  steinhart /= thermoBeta;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (thermoOhmTemp + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                  // Invert
  steinhart -= 273.15;                          // convert absolute temp to C
}

void printTimer() {
  //Print time elapsed
  elapsedTime = millis() - startTime;
  timer = elapsedTime / 1000;
  timeMinutes = timer / 60;
  timeSeconds = timer % 60;
  if (timer > prev_timeSeconds) {
    Serial.print(timeMinutes);
    Serial.print("m ");
    Serial.print(timeSeconds);
    Serial.print("s, ");
    Serial.print(steinhart - (targetTemp - 65));
    Serial.print(", ");
    for (int i = 0; i < TCScount; i++) {
      Serial.print(currentReadings[i]);
      Serial.print(pos_count[i]);
      Serial.print(", ");
       if (i < TCScount - 1) {
        Serial.print(pos_count[i]);
        Serial.print(", ");
      }
      else {
        Serial.print(pos_count[i]);
      }
    }
    prev_timeSeconds = timer;
  }
}

void endScreen() {
  //Tell user completion and turn both lights on
  Serial.print("Process Complete!");
  delay(500);
  analogWrite(fetPin, 0);
  while (1) {}
}

void RGBsensor(uint8_t sensorNum) {
  tcaselect(sensorNum);

  //RGB
  rgb[0] = tcs[sensorNum].read16(TCS34725_RDATAL);
  rgb[1] = tcs[sensorNum].read16(TCS34725_GDATAL);
  rgb[2] = tcs[sensorNum].read16(TCS34725_BDATAL);

  //XYZ and CIV
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      XYZ[i] += patchMatrix[i][j] * rgb[j];
    }
    if (i == 1) {
      CIV = XYZ[i];
    }
    if (i == 2) {
      CIV -= XYZ[i];
    }
    XYZ[i] = 0;
  }

  //Normalise CIV
  if (!initialCIVfound[sensorNum]) {
    initialCIV[sensorNum] = CIV;
    initialCIVfound[sensorNum] = true;
  }
  zeroCIV = CIV - initialCIV[sensorNum];

  //Determine moving average
  sum -= readings[index];             //removes oldest entry
  readings[index] = zeroCIV;          //Add newest to window
  sum += zeroCIV;                     //Add newest to sum
  index = (index + 1) % window_size;  //Increment the index or wrap to 0
  averaged = sum / window_size;

  //Check for increase in CIV
  uint8_t previousIndex = currentIndex % numReadings;
  currentReadings[sensorNum] = averaged;

  float diff = currentReadings[sensorNum] - (pos_readings[sensorNum][previousIndex] + 0.5) / numReadings;

  if (timeMinutes >= 5 && diff > 0) {
    pos_count[sensorNum]++;
  } else {
    pos_count[sensorNum] = 0;
  }

  pos_readings[sensorNum][currentIndex] = currentReadings[sensorNum];
  currentIndex = (currentIndex + 1) % numReadings;
}