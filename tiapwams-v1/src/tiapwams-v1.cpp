/* 
 * Project myProject
 * Author: Your Name
 * Date: 
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


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);


// pins
const int BUTTONPIN = D15;
const int RELAYPIN = D16;

// constants
const int PIXELCOUNT = 12;
const int BRIGHTNESS = 50;
const int OLED_RESET = -1;
const int BME280_HEX_ADDRESS = 0x76;
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;

// variables
bool pressed;
int pixelColor;
int status;
float tempC;
float pressPA;
float humidRH;

// objects
Button button(BUTTONPIN);

Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;


// setup() 
void setup() {

  // set up pins (do the pinMode)
  pinMode(RELAYPIN, OUTPUT);

  // set up serial monitor
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);
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
  status = bme.begin(BME280_HEX_ADDRESS);
  if (!status) {
    Serial.printf("BME280 at address 0x%02X failed to start.\n", BME280_HEX_ADDRESS);
  }
  tempC = bme.readTemperature();
  pressPA = bme.readPressure();
  humidRH = bme.readHumidity();

  // set up WiFi
  WiFi.on();
  WiFi.clearCredentials();
  WiFi.setCredentials("IoTNetwork");
  WiFi.connect();
  int wifiCount = 0;
  while (!WiFi.ready() && wifiCount <= 20) {
    Serial.printf(".");
    delay(1000);
    wifiCount++;
  }
  Serial.printf("\n\n");

  // initialize variables
  pressed = false;
}


// loop() 
void loop() {

  // run the pump while button is pressed
  while (button.isPressed()) {
    digitalWrite(RELAYPIN, HIGH);
    if (!pressed) {
      Serial.printf("Button is pressed!\n");
      pressed = true;
    }
  }
  digitalWrite(RELAYPIN, LOW);
  if (pressed) {
    Serial.printf("Button is not pressed.\n");
    pressed = false;
  }

}
