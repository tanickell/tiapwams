/* 
 * Project Tiapwams - Technologic IoT (Totally Insane!) Automated Plant Watering and Monitoring System
 * Description: Midterm 2 Project (Plant Watering System) for Deep Dive IoT Bootcamp Cohort 17
 * Author: Tim Nickell
 * Date: 2025-11-12
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */


// Include Particle Device OS APIs
#include "Particle.h"
#include "IoTClassroom_CNM.h"
#include "neopixel.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "HX711.h"
#include "Grove_Air_quality_Sensor.h"
#include "math.h"
#include "credentials.h"


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

// Global State
TCPClient TheClient;

// set up MQTT client class by passing in the WiFI client and login details
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feeds
Adafruit_MQTT_Subscribe buttonSubFeed       = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/watering-button-feed");
Adafruit_MQTT_Publish   scalePubFeed        = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/weights");
Adafruit_MQTT_Publish   moisturePubFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soil-moisture-feed");
Adafruit_MQTT_Publish   aqValuePubFeed      = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality-value-feed");
Adafruit_MQTT_Publish   aqMessagePubFeed    = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality-message-feed");
Adafruit_MQTT_Publish   temperaturePubFeed  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme-temperature-feed");
Adafruit_MQTT_Publish   pressurePubFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme-pressure-feed");
Adafruit_MQTT_Publish   humidityPubFeed     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme-humidity-feed");
Adafruit_MQTT_Publish   totalMessagePubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/total-message-feed");

// Pins
const int SCALEDTPIN = D8;
const int SCALECLKPIN = D9;
const int AQSENSORPIN = D11;
const int MOISTUREPIN = D12;
const int BUTTONPIN = D15;
const int RELAYPIN = D16;

// Constants
const int SERIAL_BEGIN = 9600;
const int SERIAL_TIMEOUT = 10000;
const int WATERING_TIME = 500; // 0.5 seconds
const int MOIST_DRY_THRESHOLD = 2000; // anything above 2000 is dry enough to run the pump

const int PIXELCOUNT = 13; // neopixels
const int BRIGHTNESS = 50;

const int OLED_RESET = -1; // OLED
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;

const int BME280_HEX_ADDRESS = 0x76; // BME
const int BME_READ_TIMER_DELAY = 10000; // make under 1000 to account for OLED delay ~ 150ms
const int BME_PUBLISH_TIMER_DELAY = 30000;
const int PRESSURE_OFFSET = 5; // 1 inch per 1000 feet

const int SUB_TIMEOUT = 100; // WiFi/MQTT

const int CALFACTOR = 425; // scale
const int SAMPLES = 10;
const int SCALE_READ_TIMER_DELAY = 10000;
const int SCALE_PUBLISH_TIMER_DELAY = 30000;

const int MOISTURE_READ_TIMER_DELAY = 10000; // moisture sensor
const int MOISTURE_PUBLISH_TIMER_DELAY = 30000;

const int AQSENSOR_READ_TIMER_DELAY = 10000; // aq sensor
const int AQSENSOR_PUBLISH_TIMER_DELAY = 30000;
const String airQualityMessages[5] = {
  "High pollution! (Force signal active.)\n",
  "High pollution!\n",
  "Low pollution.\n",
  "Fresh air.\n",
  "Something went wrong.\n"
};

const int DATA_PUBLISH_DELAY = 30000;

// Variables
bool pressed; // button

int pixelColor; // neopixels

int bmestatus; // bme 
float tempC;
float pressPA;
float humidRH;
float tempF;
float pressInHg;

float weight, rawData, calibration; // scale
int offset;
unsigned int last, lastTime;

int moisture; // moisture sensor

unsigned long duration;
unsigned long lowpulseoccupancy;
float ratio;
float concentration;

// Objects
Button button(BUTTONPIN);
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;
HX711 myScale(SCALEDTPIN, SCALECLKPIN);
AirQualitySensor aqsensor(AQSENSORPIN);

IoTTimer subTimer;
IoTTimer wateringTimer;
IoTTimer bmeReadTimer;
IoTTimer bmePublishTimer;
IoTTimer moistureReadTimer;
IoTTimer moisturePublishTimer;
IoTTimer scaleReadTimer;
IoTTimer scalePublishTimer;
IoTTimer aqsensorReadTimer;
IoTTimer aqsensorPublishTimer;

// Functions
void MQTT_connect();
bool MQTT_ping();
void pixelFill(int start, int end, int color);
float celsiusToFahrenheit(float celsius);
float pascalsToInHg(float pascals);


// setup() 
void setup() {

  // set up pins (do the pinMode)
  pinMode(MOISTUREPIN, INPUT);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(D7, OUTPUT);

  digitalWrite(D7, LOW);

  // set up serial monitor
  Serial.begin(SERIAL_BEGIN);
  waitFor(Serial.isConnected, SERIAL_TIMEOUT);
  delay(1000);
  Serial.printf("Ready to go!\n\n");

  // set up neopixels
  pixelColor = 0;
  pixel.begin();
  pixel.setBrightness(BRIGHTNESS);
  pixel.show();

  // initialize OLED & display splash
  display.begin();
  display.display();
  delay(3000);
  display.clearDisplay();

  // initialize BME280
  bmestatus = bme.begin(BME280_HEX_ADDRESS);
  if (!bmestatus) {
    Serial.printf("BME280 at address 0x%02X failed to start.\n", BME280_HEX_ADDRESS);
  }
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  humidRH = bme.readHumidity();

  // set up WiFi (connect to internet but not particle cloud)
  WiFi.on();
  WiFi.connect();
  while (WiFi.connecting()) {
    Serial.printf(".");
    delay(1000);
  }
  Serial.printf("\n\n");

  // set up MQTT subscription
  mqtt.subscribe(&buttonSubFeed);

  // set up scale
  myScale.set_scale();          // initialize loadcell
  delay(5000);                  // let the loadcell settle
  myScale.tare();               // set the tare weight (or zero)
  myScale.set_scale(CALFACTOR); // adjust when calibrating scale to desired units

  // initialize the air quality sensor
  Serial.printf("Waiting for sensor to init...\n");
  delay(20000);
  if (aqsensor.init()) {
    Serial.printf("Sensor ready!\n");
  }
  else {
    Serial.printf("Sensor ERROR!\n");
  }

  // timers
  subTimer.startTimer(0);
  wateringTimer.startTimer(0);
  bmeReadTimer.startTimer(0);
  bmePublishTimer.startTimer(0);
  moistureReadTimer.startTimer(0);
  moisturePublishTimer.startTimer(0);
  scaleReadTimer.startTimer(0);
  scalePublishTimer.startTimer(0);
  aqsensorReadTimer.startTimer(0);
  aqsensorPublishTimer.startTimer(0);

  // initialize variables
  pressed = false;

  ratio = 0;
  concentration = 0;
  lowpulseoccupancy = 0;
}


// loop() 
void loop() {

  // connect to and ping mqtt
  MQTT_connect();
  MQTT_ping();

  // manual mode: run the pump while button is pressed
  while (button.isPressed()) {
    digitalWrite(RELAYPIN, HIGH);
    if (!pressed) {
      Serial.printf("Button is pressed!\n");
      pressed = true;
    }
    delay(100);
  }
  if (pressed) {
    digitalWrite(RELAYPIN, LOW);
    Serial.printf("Button is not pressed.\n");
    pressed = false;
    delay(100);
  }

  // otherwise, button is not pressed --> sensors and cloud!
  
  // 1. subscribe to Adafruit cloud subscription(s) and wait for cloud commands
  // 2. publish monitoring data to Adafruit cloud console
  // 3. stretch: program watering schedule


  // subscribe to button press
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(SUB_TIMEOUT))) {
    // check sub 1
    if (subscription == &buttonSubFeed) {
      int subValue = atoi((char *)buttonSubFeed.lastread);
      if (subValue == 1) {
        digitalWrite(RELAYPIN, HIGH);
      }
      else {
        digitalWrite(RELAYPIN, LOW);
      }
    }
  }

  // bme
  if (bmeReadTimer.isTimerReady()) {

    tempC = bme.readTemperature();
    pressPA = bme.readPressure();
    humidRH = bme.readHumidity();
    tempF = celsiusToFahrenheit(tempC);
    pressInHg = pascalsToInHg(pressPA) + PRESSURE_OFFSET;
    Serial.printf("tempF\n%0.2f\n\npressInHg\n%0.2f\n\nhumidRH\n%0.2f\n\n", 
      tempF, pressInHg, humidRH);

    // print same to OLED display:
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.printf("tempF  |  %0.2f\n\npressInHg  |  %0.2f\n\nhumidRH  |  %0.2f\n\n", 
      tempF, pressInHg, humidRH);
    display.display();

    // PUBLISH HERE
    if (bmePublishTimer.isTimerReady()) {
      temperaturePubFeed.publish(tempF);
      pressurePubFeed.publish(pressInHg);
      humidityPubFeed.publish(humidRH);
      Serial.printf("Publishing tempF, pressInHg, humidRH: %0.2f %0.2f %0.2f\n",
        tempF, pressInHg, humidRH);
      bmePublishTimer.startTimer(BME_PUBLISH_TIMER_DELAY);
    }
    bmeReadTimer.startTimer(BME_READ_TIMER_DELAY); // update clock on OLED a little faster than once/sec to account for render lag
  }

  // scale
  if (scaleReadTimer.isTimerReady()) {
    weight = myScale.get_units(SAMPLES); // return weight in units set by set_scale()
    Serial.printf("Weight (in grams) --> %0.2f\n", weight);
    scaleReadTimer.startTimer(SCALE_READ_TIMER_DELAY);
    // PUBLISH HERE
    if (scalePublishTimer.isTimerReady()) {
      scalePubFeed.publish(weight);
      Serial.printf("Publishing weight value %0.2f\n", weight);
      scalePublishTimer.startTimer(SCALE_PUBLISH_TIMER_DELAY);
    }
  }

  // moisture sensor
  if (moistureReadTimer.isTimerReady()) {
    moisture = analogRead(MOISTUREPIN);

    // automated watering based on moisture reading
    if (moisture > MOIST_DRY_THRESHOLD) {
      digitalWrite(RELAYPIN, HIGH);
      wateringTimer.startTimer(WATERING_TIME);
      while (!wateringTimer.isTimerReady()) {
        // do nothing
      }
      digitalWrite(RELAYPIN, LOW);
    }
    // if (wateringTimer.isTimerReady()) {
    //   digitalWrite(RELAYPIN, LOW);
    // }

    moistureReadTimer.startTimer(MOISTURE_READ_TIMER_DELAY);
    Serial.printf("Moisture: %d\n", moisture);
    // PUBLISH HERE
    if (moisturePublishTimer.isTimerReady()) {
      moisturePubFeed.publish(moisture);
      Serial.printf("Publishing moisture value %d\n", moisture);
      moisturePublishTimer.startTimer(MOISTURE_PUBLISH_TIMER_DELAY);
    }
  }

  // air quality sensor
  if (aqsensorReadTimer.isTimerReady()) {
    int quality = aqsensor.slope();
    int aqValue = aqsensor.getValue();
    String pubMessage = "";

    Serial.printf("Sensor value: %d\n", aqValue);
    if (quality == AirQualitySensor::FORCE_SIGNAL) {
      pubMessage = airQualityMessages[0];
      pixelFill(0, PIXELCOUNT - 1, red);
    }
    else if (quality == AirQualitySensor::HIGH_POLLUTION) {
      pubMessage = airQualityMessages[1];
      pixelFill(0, PIXELCOUNT - 1, yellow);
    }
    else if (quality == AirQualitySensor::LOW_POLLUTION) {
      pubMessage = airQualityMessages[2];
      pixelFill(0, PIXELCOUNT - 1, green);
    }
    else if (quality == AirQualitySensor::FRESH_AIR) {
      pubMessage = airQualityMessages[3];
      pixelFill(0, PIXELCOUNT - 1, blue);
    }
    else {
      pubMessage = airQualityMessages[4];
      pixelFill(0, PIXELCOUNT - 1, violet);
    }
    Serial.printf(pubMessage);

    if (aqsensorPublishTimer.isTimerReady()) {
      if (mqtt.Update()) {
        aqValuePubFeed.publish(aqValue);
        Serial.printf("Publishing aqValue %d\n", aqValue);
        aqMessagePubFeed.publish(String::format("%s\n", pubMessage.c_str()).c_str());
        Serial.printf("Publishing %s\n", pubMessage.c_str());
      }
      aqsensorPublishTimer.startTimer(AQSENSOR_PUBLISH_TIMER_DELAY);
    }
    aqsensorReadTimer.startTimer(AQSENSOR_READ_TIMER_DELAY);
  }
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}


// ping MQTT
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

void pixelFill(int start, int end, int color) {
  pixel.clear();
  for (int i = start; i <= end; i++) {
    pixel.setPixelColor(i, color);
  }
  pixel.show();
}

float celsiusToFahrenheit(float celsius) {
  return (9.0 / 5.0) * celsius + 32;
}

float pascalsToInHg(float pascals) {
  return (1.0 / 3386.39) * pascals;
}