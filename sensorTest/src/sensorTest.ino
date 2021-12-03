/*
 * Project sensorTest
 * Description: Testing various sensors
 * Author: Casey
 * Date: 11/29/21
 */

//DEV Branch



#include <OneWire.h>
#include "DS18.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include <Adafruit_BME280.h>
#include <credentials.h>

const int PHpin = 19;
const int TempPin = 18;
const int TDSpin = 17;
const int pHRelayPin = 10;
const int nutrientRelayPin = 11;

Adafruit_BME280 bme;
DS18 tempSensor(TempPin);
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

Adafruit_MQTT_Publish TDSfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.tdsfeed");
Adafruit_MQTT_Publish pHfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.phfeed");
Adafruit_MQTT_Publish wetTempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.wettempfeed");
Adafruit_MQTT_Publish airTempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.airtempfeed");
Adafruit_MQTT_Publish airPressureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.airpressurefeed");
Adafruit_MQTT_Publish airHumidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.airhumidityfeed");

Adafruit_MQTT_Subscribe pHdown = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.phdown");
Adafruit_MQTT_Subscribe nutrientAdd = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/hydroponic-data.nutrientadd");

#define VREF 5.0      
#define SCOUNT  30          
int analogBuffer[SCOUNT];    
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

float tempC, tempF, pressPA, pressInHg, relHumid;
float waterTemp;

float pH;

unsigned long last, lastUpdate, lastAmend;

SYSTEM_MODE(SEMI_AUTOMATIC);

void setup() {
  WiFi.connect();
  Serial.begin(9600);
  mqtt.subscribe(&pHdown);
  mqtt.subscribe(&nutrientAdd);
  pinMode(PHpin, INPUT);
  pinMode(TDSpin, INPUT);
  bme.begin();  
}
void loop() {

  MQTT_connect();

  wetTemp();
  TDSsensor();
  PHsensor();
  bmeCheck();

  if((millis()-lastUpdate)>30000){
    mqttUpdate();
    lastUpdate = millis();
  }

  if((millis()-lastAmend)>1200000){
    amendPh();
    amendNutrients();
    lastAmend = millis();
  }

  mqttPing();

}

void PHsensor () { // After seemingly working well yesterday, readings are all over the place today. 
  int SAMPLES = 5000;
  int sensorValue = 0;
  long sensorSum = 0;
  float sensorAverage = 0;
  //Serial.printf("pHread = %i\n", analogRead(PHpin));
  for (int i=0; i<SAMPLES; i++) {
    sensorValue = analogRead(PHpin);
    sensorSum += sensorValue;
  }
  sensorAverage = sensorSum/SAMPLES;
  pH = (0.0008*sensorAverage+24.8);
  //Serial.printf("Water pH = %.3f, pHsensorData = %f\n", pH, sensorAverage);
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
    // Serial.print("TDS-Value:");
    // Serial.print(tdsValue, 0);
    // Serial.println("ppm");
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

void wetTemp () {
  if (tempSensor.read()) {
    waterTemp = tempSensor.fahrenheit();
    //Serial.printf(" Water Temperature = %.2f F\n", tempSensor.fahrenheit());
  }
}

void bmeCheck (void) {
  tempC = bme.readTemperature();
  tempF = map(tempC,0.0,100.0,32.0,212.0);
  pressPA = bme.readPressure();
  pressInHg = (pressPA*0.00029530);
  relHumid = bme.readHumidity();
  // Serial.printf("Air Temp = %.2f, Relative Humidity = %.2f, Air Pressure = %.2f InHG\n", tempF, relHumid, pressInHg);
}

void mqttUpdate (void) {
  if(mqtt.Update()) {
      pHfeed.publish(pH);
      TDSfeed.publish(tdsValue);
      wetTempFeed.publish(waterTemp);
      airTempFeed.publish(tempF);
      airPressureFeed.publish(pressInHg);
      airHumidityFeed.publish(relHumid);
  }
 
  Serial.print("Water TDS = ");
  Serial.print(tdsValue, 0);
  Serial.print(" ppm, ");
  Serial.printf("Water pH = %.3f, Water Temperature = %.2f F\n", pH, waterTemp);
  Serial.printf("Air Temp = %.2f, Relative Humidity = %.2f, Air Pressure = %.2f InHG\n\n", tempF, relHumid, pressInHg);
}

void amendPh () {
  if (pH > 6.5) {
    phPump();
  }
}

void amendNutrients () {
  if (tdsValue < 650) {
    nutrientPump();
  }
} 

void mqttPHdown (void) {
  bool adjustPH;
  Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(1000))) {
      if (subscription == &pHdown) {
       adjustPH = atof((char *)pHdown.lastread);
        if (adjustPH) {
          phPump();
        }
    }
  }
}

void mqttNutrientAdd (void) {
  bool addNutrient;
  Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(1000))) {
      if (subscription == &nutrientAdd) {
       addNutrient = atof((char *)nutrientAdd.lastread);
        if (addNutrient) {
          nutrientPump();
        }
    }
  }
}

void phPump (void) {
  digitalWrite(pHRelayPin, HIGH);
  delay(1000);
  digitalWrite(pHRelayPin, LOW);
  Serial.printf("Added pH Down\n");
}

void nutrientPump (void) {
  digitalWrite(nutrientRelayPin, HIGH);
  delay(1000);
  digitalWrite(nutrientRelayPin, LOW);
  Serial.printf("Added Nutrients\n");
}



// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");
}

// Ping MQTT Broker every 2 minutes to keep connection alive
void mqttPing (void) {
   if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
   }
}