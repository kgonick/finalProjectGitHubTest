// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(30, 32, NEO_RGB);

int mapXPin = 0;
int mapYPin = 0;
int mapZPin = 0;
int powerSwitch = 33;

void setup(void) {
  pinMode(powerSwitch, INPUT);

  neopixel.begin();
  neopixel.clear();
  neopixel.show();
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
    case ICM20948_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case ICM20948_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case ICM20948_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case ICM20948_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  Serial.println("OK");

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);
}

void loop() {
  if (digitalRead(powerSwitch) == HIGH){
    runProgram();
  } else {
    turnOff();
  }

//  runProgram();
}




void runProgram() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  /* Display the results (acceleration is measured in m/s^2) */
  //  Serial.print("\t\tAccel X: ");
  //  Serial.print((int)(accel.acceleration.x * 100));
  //  Serial.print(" \tY: ");
  //  Serial.print((int)(accel.acceleration.y * 100));
  //  Serial.print(" \tZ: ");
  //  Serial.print((int)(accel.acceleration.z * 100));
  //  Serial.println(" m/s^2 ");
  
  mapYPin = map(accel.acceleration.y * 100, -1100, 1100, 0, 255);
  mapYPin = constrain(mapYPin, 10, 255);
  mapXPin = map(accel.acceleration.x * 100, -1100, 1100, 0, 255);
  mapXPin = constrain(mapXPin, 10, 255);
  mapZPin = map(accel.acceleration.z * 100, -1100, 1100, 0, 255);
  mapZPin = constrain(mapZPin, 10, 255);
  Serial.println(mapYPin);
  Serial.println( );
  Serial.println(mapXPin);
  Serial.println( );
  Serial.println(mapZPin);
  for (int i = 0; i < 30; i++) {
    neopixel.setPixelColor(i, 0, mapXPin, mapYPin);
  }
  neopixel.show();

  delay(100);
}



void turnOff() {
  for (int i = 0; i < 30; i++) {
    neopixel.setPixelColor(i, 0, 0, 0);
  }
  neopixel.show() ;
}
