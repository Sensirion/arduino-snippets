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

// union construct to help to convert byte array to float
union u_tag {
 byte b[4];
 float fval;
} u;

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

  float pm1p0_f, pm2p5_f, pm4p0_f, pm10p0_f;
  int16_t voc, humidity, temperature;
  uint8_t data[40], counter;

  // send read measurement data command to get pm values in floating point
  Wire.beginTransmission(SEN44_ADDRESS);
  Wire.write(0x03);
  Wire.write(0x31);
  Wire.endTransmission();
  
  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // read measurement data SEN44, after two bytes a CRC follows
  Wire.requestFrom(SEN44_ADDRESS, 40);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // only the first four float values are used
  // rest will be ignored
  // convert byte arrays to float
  u.b[0] = data[4];
  u.b[1] = data[3];
  u.b[2] = data[1];
  u.b[3] = data[0];
  pm1p0_f = u.fval;

  u.b[0] = data[10];
  u.b[1] = data[9];
  u.b[2] = data[7];
  u.b[3] = data[6];
  pm2p5_f = u.fval;

  u.b[0] = data[16];
  u.b[1] = data[15];
  u.b[2] = data[13];
  u.b[3] = data[12];
  pm4p0_f = u.fval;  

  u.b[0] = data[22];
  u.b[1] = data[21];
  u.b[2] = data[19];
  u.b[3] = data[18];
  pm10p0_f = u.fval;

  Serial.print(pm1p0_f);
  Serial.print("\t");
  Serial.print(pm2p5_f);
  Serial.print("\t");
  Serial.print(pm4p0_f);
  Serial.print("\t");
  Serial.print(pm10p0_f);
  Serial.print("\t");  

  // send read measurement data command to get VOC, RH, T
  Wire.beginTransmission(SEN44_ADDRESS);
  Wire.write(0x03);
  Wire.write(0xA6);
  Wire.endTransmission();
  
  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // read measurement data SEN44, after two bytes a CRC follows
  Wire.requestFrom(SEN44_ADDRESS, 9);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // VOC level is a signed int and scaled by a factor of 10 and needs to be divided by 10
  // humidity is a signed int and scaled by 100 and need to be divided by 100
  // temperature is a signed int and scaled by 200 and need to be divided by 200
  voc = (uint16_t)data[0] << 8 | data[1];
  humidity = (uint16_t)data[3] << 8 | data[4];
  temperature = (uint16_t)data[6] << 8 | data[7];

  Serial.print(String(float(voc) / 10));
  Serial.print("\t");
  Serial.print(String(float(humidity) / 100));
  Serial.print("\t");
  Serial.print(String(float(temperature) / 200));
  Serial.println();

  delay(10);

  // wait 1 s for next measurement
  delay(1000);
}
