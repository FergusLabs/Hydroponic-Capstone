/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/School/Documents/IoT/Hydroponic-Capstone/sensorTest/src/sensorTest.ino"
/*
 * Project sensorTest
 * Description: Testing various sensors
 * Author: Casey
 * Date: 11/29/21
 */

//DEV Branch

#include <OneWire.h>
#include "DS18.h"

void setup();
void loop();
void PHsensor ();
void TDSsensor ();
int getMedianNum(int bArray[], int iFilterLen);
void getTemp ();
#line 13 "c:/Users/School/Documents/IoT/Hydroponic-Capstone/sensorTest/src/sensorTest.ino"
DS18 tempSensor(18);

const int PHpin = 19;
const int TDSpin = 17;

#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, Celsius = 0, Fahrenheit = 0, temperature = 25;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
  Serial.begin(9600);
  pinMode(PHpin, INPUT);
  pinMode(TDSpin, INPUT);
}
void loop() {
  // PHsensor();
  // TDSsensor();
  getTemp();
}

void PHsensor () { // need to adjust, average to get better readings
  int SAMPLES = 5000;
  int sensorValue, i;
  float pH;
  for (i=0; i<SAMPLES; i++) {
    sensorValue = analogRead(PHpin);
    sensorValue += sensorValue;
  }
  sensorValue = sensorValue/SAMPLES;
  pH = 0.008*sensorValue+24.8;
  Serial.printf("sensorData = %i, pH = %f\n", sensorValue, pH);
}

void TDSsensor () {   // Readings are in range, but need calibration
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC 
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDSpin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4095.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    Serial.print("TDS----Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
}
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void getTemp () {
  if (tempSensor.read()) {
    Serial.printf("Temperature %.2f C %.2f F ", tempSensor.celsius(), tempSensor.fahrenheit());
  }
}