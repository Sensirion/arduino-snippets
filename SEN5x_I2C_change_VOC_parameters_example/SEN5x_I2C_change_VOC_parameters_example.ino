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

// SEN5x
const int16_t SEN55_ADDRESS = 0x69;

void setup() {
  int16_t voc_offset, voc_learning, voc_learning_gain, voc_gating, voc_initial, voc_gain;
  uint8_t data[18], counter;

  Serial.begin(115200);

  // Wait for serial connection from PC
  // Comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // Wait until sensors startup, > 50 ms according to datasheet
  delay(50);

  // Initiate I2C communication
  Wire.begin();

  // Send command to read preset VOC algorithm tuning parameters (0x60D0)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xD0);
  Wire.endTransmission();

  // Wait 20 ms to allow for command execution
  delay(20);

  // Read preset VOC algorithm tuning parameters from SEN55
  Wire.requestFrom(SEN55_ADDRESS, 18);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // Processing preset VOC algorithm tuning parameters
  // CRC byte (every 3rd byte) is excluded from processing
  // offset is arbitrary
  voc_offset = (uint16_t)data[0] << 8 | data[1];
  // Learning time is in hours
  voc_learning = (uint16_t)data[3] << 8 | data[4];
  // Learning time gain in hours
  voc_learning_gain = (uint16_t)data[6] << 8 | data[7];
  // Gating time is in minutes
  voc_gating = (uint16_t)data[9] << 8 | data[10];
  // Standard initial is arbitrary
  voc_initial = (uint16_t)data[12] << 8 | data[13];  
  // Standard initial is arbitrary
  voc_gain = (uint16_t)data[15] << 8 | data[16]; 
  
  Serial.println();
  Serial.println("Default parameter values ");
  Serial.print("Index offset: ");
  Serial.println(voc_offset);
  Serial.print("Learning time offset hours: ");
  Serial.println(voc_learning);
  Serial.print("Learning time gain hours: ");
  Serial.println(voc_learning_gain);
  Serial.print("Gating max duration minutes: ");
  Serial.println(voc_gating);
  Serial.print("Std initial: ");
  Serial.println(voc_initial);
  Serial.print("Gain factor: ");
  Serial.println(voc_gain);
  Serial.println();

  // Set new VOC algorithm tuning parameters
  // Offset 250 instead of 100
  voc_offset = 250;
  // Learning 6 h instead of 12 h
  voc_learning = 6;
  // Learning gain 6 h instead of 12 h
  voc_learning_gain = 6;
  // Gating 60 min instead of 180 min
  voc_gating = 60;
  // Initial to 60 instead of 50
  voc_initial = 60;
  // Gain 200 instead of 230
  voc_gain = 200;
  
  // Parsing
  data[0] = (voc_offset & 0xff00) >> 8;
  data[1] = voc_offset & 0x00ff;
  data[2] = CalcCrc(data);
  data[3] = (voc_learning & 0xff00) >> 8;
  data[4] = voc_learning & 0x00ff;
  data[5] = CalcCrc(data+3);
  data[6] = (voc_learning_gain & 0xff00) >> 8;
  data[7] = voc_learning_gain & 0x00ff;
  data[8] = CalcCrc(data+6);
  data[9] = (voc_gating & 0xff00) >> 8;
  data[10] = voc_gating & 0x00ff;
  data[11] = CalcCrc(data+9);
  data[12] = (voc_initial & 0xff00) >> 8;
  data[13] = voc_initial & 0x00ff;
  data[14] = CalcCrc(data+12);
  data[15] = (voc_gain & 0xff00) >> 8;
  data[16] = voc_gain & 0x00ff;
  data[17] = CalcCrc(data+15);

  // Send new VOC parameters values to sensor (will be held in RAM, not persistent)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xD0);
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
  Wire.write(data[12]);
  Wire.write(data[13]);
  Wire.write(data[14]);
  Wire.write(data[15]);
  Wire.write(data[16]);
  Wire.write(data[17]);
  Wire.endTransmission();

  // Wait 20 ms to allow for command execution
  delay(20);


  // Send command to read new VOC algorithm tuning parameters
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xD0);
  Wire.endTransmission();
  
  // Wait 20 ms to allow for command execution
  delay(20);

  // Read VOC algorithm tuning parameters from SEN55
  Wire.requestFrom(SEN55_ADDRESS, 18);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }


  // Parse data to make sure that new VOC algorithm tuning parameters are correct
  // offset is arbitrary
  voc_offset = (uint16_t)data[0] << 8 | data[1];
  // learning time is in hours
  voc_learning = (uint16_t)data[3] << 8 | data[4];
  // learning time gain in hours
  voc_learning_gain = (uint16_t)data[6] << 8 | data[7];
  // gating time is in minutes
  voc_gating = (uint16_t)data[9] << 8 | data[10];
  // standard initial is arbitrary
  voc_initial = (uint16_t)data[12] << 8 | data[13];  
  // standard initial is arbitrary
  voc_gain = (uint16_t)data[15] << 8 | data[16]; 

  // Print new VOC algorithm tuning parameters
  Serial.println();
  Serial.println("New parameter values");
  Serial.print("Index offset: ");
  Serial.println(voc_offset);
  Serial.print("Learning time offset hours: ");
  Serial.println(voc_learning);
  Serial.print("Learning time gain hours: ");
  Serial.println(voc_learning_gain);
  Serial.print("Gating max duration minutes: ");
  Serial.println(voc_gating);
  Serial.print("Std initial: " );
  Serial.println(voc_initial);
  Serial.print("Gain factor: ");
  Serial.println(voc_gain);
  Serial.println();
  
  // Send command to start measurement (0x0021)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();  

  // Wait until command is executed, sensors are ready and fan is initialized
  delay(2000);

  // Output measurement value format
  Serial.println();
  Serial.println("PM1.0\tPM2.5\tPM4.0\tPM10.0\tVOC_Index\tNOx_Index\tRH\tT");  
}

void loop() {
  
  uint16_t pm1p0, pm2p5, pm4p0, pm10p0;
  int16_t voc, nox, humidity, temperature;
  uint8_t data[24], counter;

  // Send command to read measurement data (0x03C4)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x03);
  Wire.write(0xC4);
  Wire.endTransmission();

  // Wait 20 ms for command execution
  delay(20);

  // Read measurement data from SEN55
  Wire.requestFrom(SEN55_ADDRESS, 24);
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
  humidity = (uint16_t)data[12] << 8 | data[13];
  temperature = (uint16_t)data[15] << 8 | data[16];
  voc = (uint16_t)data[18] << 8 | data[19];
  nox = (uint16_t)data[21] << 8 | data[22];

  // Begin measurement output
  Serial.print(String(float(pm1p0) / 10));
  Serial.print("\t");
  Serial.print(String(float(pm2p5) / 10));
  Serial.print("\t");
  Serial.print(String(float(pm4p0) / 10));
  Serial.print("\t");
  Serial.print(String(float(pm10p0) / 10));
  Serial.print("\t");
  Serial.print(String(float(voc) / 10));
  Serial.print("\t\t");
  Serial.print(String(float(nox) / 10));
  Serial.print("\t\t");
  Serial.print(String(float(humidity) / 100));
  Serial.print("\t");
  Serial.print(String(float(temperature) / 200));
  Serial.println();

  // Wait 1 s for next measurement
  delay(1000);
}

// Calculate CRC according to datasheet
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
