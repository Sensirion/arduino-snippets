/*
 * Copyright (c) 2020, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <Wire.h>

// SEN63C
const int16_t SEN66_ADDRESS = 0x6B;

void setup() {
  Serial.begin(115200);
  // Wait for serial connection from PC
  // Comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // Wait until sensors startup, > 50 ms according to datasheet
  delay(50);
  
  // Initiate I2C communication
  Wire.begin();

  // Send command to start measurement (0x0021)
  Wire.beginTransmission(SEN66_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();

  Serial.println("PM1.0\tPM2.5\tPM4.0\tPM10.0\tRH\tT\tVOC\tNOX\tCO2");
  
  // Wait until sensor is ready, fan is initialized
  delay(1000);
}



void loop() {

  // Initializing variables
  uint16_t pm1p0, pm2p5, pm4p0, pm10p0;
  int16_t co2, voc, nox, humidity, temperature;
  uint8_t data[27], counter;

  // Send command to read measurement data (0x0471)
  Wire.beginTransmission(SEN66_ADDRESS);
  Wire.write(0x03);
  Wire.write(0x00);
  Wire.endTransmission();
  // Wait 20 ms to allow the sensor to fill the internal buffer
  delay(20);

  // Read measurement data of SEN55, after two bytes a CRC follows
  Wire.requestFrom(SEN66_ADDRESS, 27);
  counter = 0;
  
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // PM1.0 to PM10 are scaled unsigned integer values in ug / um3 and need to be divided by 10
  // humidity is a signed int and scaled by 100 and need to be divided by 100
  // temperature is a signed int and scaled by 200 and need to be divided by 200
  // CO2 concentration a signed int and not scaled, during sensor init value is the first 30s value is 0x7fff/ 32767 
  pm1p0 = (uint16_t)data[0] << 8 | data[1];
  pm2p5 = (uint16_t)data[3] << 8 | data[4];
  pm4p0 = (uint16_t)data[6] << 8 | data[7];
  pm10p0 = (uint16_t)data[9] << 8 | data[10];
  humidity = (uint16_t)data[12] << 8 | data[13];
  temperature = (uint16_t)data[15] << 8 | data[16];
  voc = (uint16_t)data[18] << 8 | data[19];
  nox = (uint16_t)data[21] << 8 | data[22];
  co2 = (uint16_t)data[24] << 8 | data[25];

  // Print output
  Serial.print(String(float(pm1p0) / 10));
  Serial.print("\t");
  Serial.print(String(float(pm2p5) / 10));
  Serial.print("\t");
  Serial.print(String(float(pm4p0) / 10));
  Serial.print("\t");
  Serial.print(String(float(pm10p0) / 10));
  Serial.print("\t");
  Serial.print(String(float(humidity) / 100));
  Serial.print("\t");
  Serial.print(String(float(temperature) / 200));
  Serial.print("\t");
  Serial.print(String(float(voc) / 10));
  Serial.print("\t");
  Serial.print(String(float(nox) / 10));
  Serial.print("\t");
  Serial.print(String(float(co2)));
  Serial.println();

  // Wait 1 s for next measurement
  delay(1000);
}
