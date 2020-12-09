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

// SVM40
const int16_t SVM40_ADDRESS = 0x6A;

void setup() {
  int16_t voc_offset,voc_learning, voc_gating, voc_initial;
  uint8_t data[12], counter;

  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // output format
  Serial.println("VOC_Index\tRH\tT");  

  // init I2C
  Wire.begin();
  
  // wait until sensors startup, > 1 ms according to datasheet
  delay(100);

  // make sure sensor is stopped to change parameters
  Wire.beginTransmission(SVM40_ADDRESS);
  Wire.write(0x01);
  Wire.write(0x04);
  Wire.endTransmission();

  // wait until sensor is ready
  delay(100);

  // read current VOC parameters from flash memory
  Wire.beginTransmission(SVM40_ADDRESS);
  Wire.write(0x60);
  Wire.write(0x83);
  Wire.endTransmission();  

  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // read offset data, after two bytes a CRC follows
  Wire.requestFrom(SVM40_ADDRESS, 12);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // values are unscaled
  // offset is arbitrary
  voc_offset = (uint16_t)data[0] << 8 | data[1];
  // learning time is in hours
  voc_learning = (uint16_t)data[3] << 8 | data[4];
  // gating time is in minutes
  voc_gating = (uint16_t)data[6] << 8 | data[7];
  // standard initial is arbitrary
  voc_initial = (uint16_t)data[9] << 8 | data[10];  Serial.println();

  Serial.println("default parameters (offset, learning, gating, initial): ");
  Serial.println(voc_offset);
  Serial.println(voc_learning);
  Serial.println(voc_gating);
  Serial.println(voc_initial);
  Serial.println();
  
  // Set new algorithm parameters
  // Offset 200 instead of 100
  voc_offset = 200;
  // Learning 6 h instead of 12 h
  voc_learning = 12;
  // gating 60 min instead of 180 min
  voc_gating = 60;
  // initial same as default with 50
  voc_initial = 50;

  // prepare buffer with algorithm parameter data
  // calculate CRC for each 2 bytes of data
  data[0] = (voc_offset & 0xff00) >> 8;
  data[1] = voc_offset & 0x00ff;
  data[2] = CalcCrc(data);
  data[3] = (voc_learning & 0xff00) >> 8;
  data[4] = voc_learning & 0x00ff;
  data[5] = CalcCrc(data+3);
  data[6] = (voc_gating & 0xff00) >> 8;
  data[7] = voc_gating & 0x00ff;
  data[8] = CalcCrc(data+6);
  data[9] = (voc_initial & 0xff00) >> 8;
  data[10] = voc_initial & 0x00ff;
  data[11] = CalcCrc(data+9);

  // send new value for voc parameters to sensor (will be hold in RAM, not persistent)
  Wire.beginTransmission(SVM40_ADDRESS);
  Wire.write(0x60);
  Wire.write(0x83);
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
  Wire.write(data[4]);
  Wire.write(data[5]);
  Wire.write(data[6]);
  Wire.write(data[7]);
  Wire.write(data[8]);
  Wire.write(data[9]);
  Wire.write(data[10]);
  Wire.write(data[11]);
  Wire.endTransmission();

  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // now send command to save parameter in the flash (NVM memory) of the sensor to have persistence
  Wire.beginTransmission(SVM40_ADDRESS);
  Wire.write(0x60);
  Wire.write(0x02);
  Wire.endTransmission();
  
  // wait 30 ms to allow the sensor to write the data into the flash
  delay(30);

  // repeat read data to make sure that the values are applied correctly
  Wire.beginTransmission(SVM40_ADDRESS);
  Wire.write(0x60);
  Wire.write(0x83);
  Wire.endTransmission();  

  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // read offset data, after two bytes a CRC follows
  Wire.requestFrom(SVM40_ADDRESS, 12);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }
  // values are unscaled
  // offset is arbitrary
  voc_offset = (uint16_t)data[0] << 8 | data[1];
  // learning time is in hours
  voc_learning = (uint16_t)data[3] << 8 | data[4];
  // gating time is in minutes
  voc_gating = (uint16_t)data[6] << 8 | data[7];
  // standard initial is arbitrary
  voc_initial = (uint16_t)data[9] << 8 | data[10];  Serial.println();

  Serial.println("new parameters (offset, learning, gating, initial): ");
  Serial.println(voc_offset);
  Serial.println(voc_learning);
  Serial.println(voc_gating);
  Serial.println(voc_initial);
  Serial.println();

  // wait 10 ms to allow the sensor to be ready again
  delay(10);
  
  // start up sensor, sensor will go to continous measurement mode
  // each second there will be new measurement values
  Wire.beginTransmission(SVM40_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x10);
  Wire.endTransmission();

  // wait until sensors is ready, fan is initialized
  delay(2000);
}

void loop() {
  int16_t voc, humidity, temperature;
  uint8_t data[9], counter;

  // read measurement data
  Wire.beginTransmission(SVM40_ADDRESS);
  Wire.write(0x03);
  Wire.write(0xA6);
  Wire.endTransmission();

  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // read measurement data sen44, after two bytes a CRC follows
  Wire.requestFrom(SVM40_ADDRESS, 9);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // VOC level is a signed int and scaled by a factor of 10 and needs to be divided by 10
  // VOC raw value is an uint16_t and has no scaling
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

  // wait 1 s for next measurement
  delay(1000);
}

// calculate CRC according to datasheet
uint8_t CalcCrc(uint8_t data[2]) {
  uint8_t crc = 0xFF;
  for(int i = 0; i < 2; i++) {
    crc ^= data[i];
    for(uint8_t bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ 0x31u;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}
