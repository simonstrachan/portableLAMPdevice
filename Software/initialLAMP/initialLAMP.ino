#include <Deque.h>              //Deque library
#include <PID_v1.h>             //PID library
#include <Wire.h>               //I2C library
#include "Adafruit_TCS34725.h"  //RGB color sensor library

//LAMP values
const double targetTemp = 82.5;
const double endTime = 30;

//Initialising Pins
#define fetPin 11
#define RD4 10
#define GR4 9
#define RD3 8
#define GR3 7
#define RD2 6
#define GR2 5
#define RD1 4
#define GR1 3
#define thermoPin A0

//Time Variables
uint32_t timer, timeMinutes, timeSeconds, elapsedTime, currentMillis, previousMillis, prev_timeSeconds, startTime;
uint16_t RGBinterval = 700;  //Time in milliseconds

//Temperature Variables
#define thermoOhm 100000       //NTC nominal resistance
#define thermoOhmTemp 25       //NTC temperature for nominal resistance
#define thermoBeta 3950        //Beta for NTC
#define seriesResistor 100000  //Value of R1
#define numSamples 250
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
float initialCIV[TCScount], positiveArray[TCScount];
float CIV, zeroCIV = 0;

// Moving average Variables
const uint8_t window_size = 15;
uint8_t index = 0;
float sum, averaged = 0;
float readings[window_size];

//Testing positive variables
const uint8_t deque_size = 60;
Deque<float> q =  Deque<float>(deque_size);
float oldCIV, newCIV = 0;
bool positive = false;
int qSize;

//Other Variables
uint8_t i, count, inChar;
bool ready, running = false;
double printArray[40];

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
      if (positiveArray[i] == 1) {
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
    Serial.print(adj_set);
    Serial.print(", ");
    for (int i = 0; i < 40; i++) {
      if (i < 39) {
        Serial.print(printArray[i]);
        Serial.print(", ");
      } else {
        Serial.println(printArray[i]);
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

  for (int i = 0; i < 3; i++) {
    printArray[sensorNum * 10 + i] = rgb[i];
  }

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
    printArray[sensorNum * 10 + i + 3] = XYZ[i];
    XYZ[i] = 0;
  }

  printArray[sensorNum * 10 + 6] = CIV;
  //Normalise CIV
  if (!initialCIVfound[sensorNum]) {
    initialCIV[sensorNum] = CIV;
    initialCIVfound[sensorNum] = true;
  }
  zeroCIV = CIV - initialCIV[sensorNum];
  printArray[sensorNum * 10 + 7] = zeroCIV;

  //Determine moving average
  sum -= readings[index];             //removes oldest entry
  readings[index] = zeroCIV;          //Add newest to window
  sum += zeroCIV;                     //Add newest to sum
  index = (index + 1) % window_size;  //Increment the index or wrap to 0
  averaged = sum / window_size;
  printArray[sensorNum * 10 + 8] = averaged;
  qSize = q.count();
  if (qSize == deque_size) {
    oldCIV = q.pop_front();
  }
  q.push_back(averaged);
  newCIV = averaged;

  if (timeMinutes >= 5 && ((newCIV - (oldCIV + 0.5)) / deque_size > 0)) {
    count++;
  } else {
    count = 0;
  }
  if (count >= 3) {
    positiveArray[sensorNum] = 1;
    printArray[sensorNum * 10 + 9] = 1;
  } else {
    printArray[sensorNum * 10 + 9] = 0;
  }
}

void tcaselect(uint8_t bus) {
  if (bus > TCScount) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << (bus + 4));
  Wire.endTransmission();
}