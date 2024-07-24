const uint8_t numReadings = 10;
float readings[2][numReadings];
float currentReadings[2];
uint8_t currentIndex, j = 0;
uint8_t count[] = { 0, 0 };
uint16_t interval = 1000;
bool readingsInc = false;

float sensorValue1[] = { 3.788, 3.791, 3.795, 3.799, 3.802, 3.808, 3.812, 3.814, 3.817, 3.82, 3.822, 3.828, 3.832, 3.836, 3.84, 3.844, 3.85, 3.856, 3.86, 3.866, 3.871, 3.877, 3.88, 3.891, 3.899 };
float sensorValue2[] = { 4.788, 4.791, 4.795, 4.799, 4.802, 4.808, 4.812, 4.814, 4.817, 4.82, 4.822, 4.828, 4.832, 4.836, 4.84, 4.844, 4.85, 4.856, 4.86, 4.866, 4.871, 4.877, 4.88, 4.891, 4.899 };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Current, Readings 1, Readings 2, Previous, Count 1, Count 2, P/N 1, P/N 2");
}

void loop() {
  // put your main code here, to run repeatedly:
  int previousIndex = currentIndex % numReadings;
  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      currentReadings[i] = sensorValue1[j];
    } else {
      currentReadings[i] = sensorValue2[j];
    }
    if (readingsInc) {
      if (((currentReadings[i] - readings[i][previousIndex]) / numReadings > 0)) {
        count[i]++;
      } else {
        count[i] = 0;
      }
    }
    readings[i][currentIndex] = currentReadings[i];
  }


  Serial.print(currentIndex);
  Serial.print(", ");
  Serial.print(readings[0][currentIndex]);
  Serial.print(", ");
  Serial.print(readings[1][currentIndex]);
  Serial.print(", ");
  Serial.print(previousIndex);
  Serial.print(", ");
  Serial.print(count[0]);
  Serial.print(", ");
  Serial.print(count[1]);
  Serial.print(", ");
  for (int i = 0; i < 2; i++) {
    if (count[i] >= 3) {
      if (i == 0) {
        Serial.print("1");
        Serial.print(", ");
      } else {
        Serial.println("1");
      }
    } else {
      if (i == 0) {
        Serial.print("0");
        Serial.print(", ");
      } else {
        Serial.println("0");
      }
    }
  }
  if (!readingsInc && currentIndex == numReadings - 1) {
    readingsInc = true;
    Serial.println("Begin readings");
  }
  currentIndex = (currentIndex + 1) % numReadings;

  j++;
  if (j == 24) {
    while (1)
      ;
  }

  //delay(interval);
}
