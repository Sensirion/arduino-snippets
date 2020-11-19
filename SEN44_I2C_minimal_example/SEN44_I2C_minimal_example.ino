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

// SEN44
const int16_t SEN44_ADDRESS = 0x69;

void setup() {
  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // output format
  Serial.println("PM1.0\tPM2.5\tPM4.0\tPM10.0\tVOC_Index\tRH\tT");
  
  // init I2C
  Wire.begin();
  
  // wait until sensors startup, > 1 ms according to datasheet
  delay(1);

  // start up sensor, sensor will go to continuous measurement mode
  // each second there will be new measurement values
  Wire.beginTransmission(SEN44_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();

  // wait until sensor is ready, fan is initialized
  delay(1000);
}

void loop() {

  uint16_t pm1p0, pm2p5, pm4p0, pm10p0;
  int16_t voc, humidity, temperature;
  uint8_t data[21], counter;

  // send read measurement data command
  Wire.beginTransmission(SEN44_ADDRESS);
  Wire.write(0x03);
  Wire.write(0x74);
  Wire.endTransmission();
  
  // wait 5 ms to allow the sensor to fill the internal buffer
  delay(5);

  // read measurement data SEN44, after two bytes a CRC follows
  Wire.requestFrom(SEN44_ADDRESS, 21);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // PM1.0 to PM10 are unscaled unsigned integer values in ug / um3
  // VOC level is a signed int and scaled by a factor of 10 and needs to be divided by 10
  // humidity is a signed int and scaled by 100 and need to be divided by 100
  // temperature is a signed int and scaled by 200 and need to be divided by 200
  pm1p0 = (uint16_t)data[0] << 8 | data[1];
  pm2p5 = (uint16_t)data[3] << 8 | data[4];
  pm4p0 = (uint16_t)data[6] << 8 | data[7];
  pm10p0 = (uint16_t)data[9] << 8 | data[10];
  voc = (uint16_t)data[12] << 8 | data[13];
  humidity = (uint16_t)data[15] << 8 | data[16];
  temperature = (uint16_t)data[18] << 8 | data[19];

  Serial.print(pm1p0);
  Serial.print("\t");
  Serial.print(pm2p5);
  Serial.print("\t");
  Serial.print(pm4p0);
  Serial.print("\t");
  Serial.print(pm10p0);
  Serial.print("\t");
  Serial.print(String(float(voc) / 10));
  Serial.print("\t");
  Serial.print(String(float(humidity) / 100));
  Serial.print("\t");
  Serial.print(String(float(temperature) / 200));
  Serial.println();

  // wait 1 s for next measurement
  delay(1000);
}
