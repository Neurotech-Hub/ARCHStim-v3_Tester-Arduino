#include <SD.h>
#include <SPI.h>
#include <Wire.h>     // Include Wire library for I2C
#include <ADS1118.h>  // [Neurotech-Hub/ADS1118-Arduino: Arduino library for TI ADS1118](https://github.com/Neurotech-Hub/ADS1118-Arduino)
#include <AD57X4R.h>  // MODIFIED [janelia-arduino/AD57X4R: Provides an SPI based interface to the AD57X4R](https://github.com/janelia-arduino/AD57X4R)

#define USB_SENSE 1
#define USER_IN 2
#define LED_B 5
#define EXT_OUTPUT 9
#define BUZZ 10
#define EXT_INPUT 11
#define ADC_CS 12
#define DAC_CS 13
#define LED_R 45
#define LED_STIM 46
#define MISO 37
#define SCK 36
#define MOSI 35
#define SD_CS 34
#define FUEL_ALERT 42
#define DISABLE 41
#define DRIVE_EN 40
#define SDA_PIN 3
#define SCL_PIN 4

const double VREF = 2.048;

ADS1118 ads1118(ADC_CS);
AD57X4R dac = AD57X4R(DAC_CS, VREF);

void setup() {
  // Set pin modes
  pinMode(USB_SENSE, INPUT);
  pinMode(USER_IN, INPUT_PULLUP);
  pinMode(LED_R, OUTPUT);
  pinMode(EXT_OUTPUT, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(EXT_INPUT, INPUT);
  pinMode(ADC_CS, OUTPUT);
  pinMode(DAC_CS, OUTPUT);
  pinMode(LED_STIM, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(FUEL_ALERT, INPUT);
  pinMode(DISABLE, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(DRIVE_EN, OUTPUT);
  digitalWrite(DRIVE_EN, LOW);  // Set DRIVE_EN for isolated components

  // Set SPI pins
  SPI.begin(SCK, MISO, MOSI, SD_CS);

  // Initial states for outputs
  digitalWrite(EXT_OUTPUT, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_STIM, LOW);
  digitalWrite(DISABLE, LOW);

  // Initialize Serial for SD card message
  Serial.begin(9600);

  // Initialize I2C with specified SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);

  // Play a 1kHz tone on the BUZZ pin (non-blocking)
  tone(BUZZ, 1000, 500);

  ads1118.begin();
  // ads1118.setSingleShotMode();
  ads1118.setSamplingRate(ads1118.RATE_128SPS);
  ads1118.setInputSelected(ads1118.AIN_0);
  ads1118.setFullScaleRange(ads1118.FSR_4096);  // !! optimize

  dac.setup(AD57X4R::AD5754R);  // !! only power up required DACs?
  dac.setAllOutputRanges(AD57X4R::BIPOLAR_5V);
  dac.setAllVoltages(0);

  digitalWrite(DISABLE, LOW);
  while (1) {
    // dac.setAnalogValue(3, 0);
    dac.setAllVoltages(0);
    delay(1000);
    digitalWrite(LED_STIM, !digitalRead(LED_STIM));
    // dac.setAnalogValue(3, DAC_MIN);
    dac.setAllVoltages(3);
    delay(1000);
    digitalWrite(LED_STIM, !digitalRead(LED_STIM));
    // dac.setAnalogValue(3, DAC_MAX);
    dac.setAllVoltages(-3);
    delay(1000);
    digitalWrite(LED_STIM, !digitalRead(LED_STIM));
  }
}

void loop() {
  // Print the state of USER_IN, EXT_INPUT, and FUEL_ALERT
  Serial.print("USB_SENSE: ");
  Serial.println(analogRead(USB_SENSE));
  Serial.print("USER_IN: ");
  Serial.println(digitalRead(USER_IN));
  Serial.print("EXT_INPUT: ");
  Serial.println(digitalRead(EXT_INPUT));

  digitalWrite(LED_B, digitalRead(FUEL_ALERT));

  // Toggle outputs
  digitalWrite(EXT_OUTPUT, !digitalRead(EXT_OUTPUT));
  digitalWrite(LED_R, !digitalRead(LED_R));
  digitalWrite(LED_STIM, !digitalRead(LED_STIM));
  digitalWrite(DISABLE, !digitalRead(DISABLE));

  // Attempt to initialize the SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!\n");
  } else {
    Serial.println("SD Card initialized successfully.\n");
  }

  // I2C Scanner
  Serial.println("Scanning I2C addresses...");
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
      delay(5);  // Small delay for stability
    }
  }

  delay(500);
}