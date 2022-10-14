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
  int16_t nox_offset, nox_learning, nox_learning_gain, nox_gating, nox_initial, nox_gain;
  uint8_t data[18], counter;

  Serial.begin(115200);

  // Wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // Initiate I2C communication
  Wire.begin();

  // Wait until sensors startup, > 50 ms according to datasheet
  delay(50);

  // Send command to NOx algorithm tuning parameters (0x60E1)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xE1);
  Wire.endTransmission();

  // Wait 20 ms to allow the sensor to fill the internal buffer
  delay(20);

  // Read preset NOx algorithm tuning parameters from SEN55
  Wire.requestFrom(SEN55_ADDRESS, 18);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // Processing preset NOx algorithm tuning parameters
  // CRC byte (every 3rd byte) is excluded from processing
  // offset is arbitrary
  nox_offset = (uint16_t)data[0] << 8 | data[1];
  // learning time is in hours
  nox_learning = (uint16_t)data[3] << 8 | data[4];
  // learning time gain in hours
  nox_learning_gain = (uint16_t)data[6] << 8 | data[7];
  // gating time is in minutes
  nox_gating = (uint16_t)data[9] << 8 | data[10];
  // standard initial is arbitrary
  nox_initial = (uint16_t)data[12] << 8 | data[13];  
  // standard initial is arbitrary
  nox_gain = (uint16_t)data[15] << 8 | data[16];  

  Serial.println();
  Serial.println("Default parameters: ");
  Serial.print("Index offset: ");
  Serial.println(float(nox_offset));
  Serial.print("Learning time offset hours: ");
  Serial.println(float(nox_learning));
  Serial.print("Learning time gain hours: ");
  Serial.println(nox_learning_gain);
  Serial.print("Gating max duration minutes: ");
  Serial.println(nox_gating);
  Serial.print("Std initial: ");
  Serial.println(nox_initial);
  Serial.print("Gain factor: ");
  Serial.println(nox_gain);
  Serial.println();

  // Set new NOx algorithm tuning parameters
  // Offset 100 instead of 1
  nox_offset = 100;
  // Learning 6 h instead of 12 h
  nox_learning = 6;
  // gating 1500 min instead of 720 min
  nox_gating = 1500;
  // gain 250 instead of 230
  nox_gain = 250;
  
  // prepare buffer with algorithm parameter data
  // calculate CRC for each 2 bytes of data
  data[0] = (nox_offset & 0xff00) >> 8;
  data[1] = nox_offset & 0x00ff;
  data[2] = CalcCrc(data);
  data[3] = (nox_learning & 0xff00) >> 8;
  data[4] = nox_learning & 0x00ff;
  data[5] = CalcCrc(data+3);
  data[6] = (nox_learning_gain & 0xff00) >> 8;
  data[7] = nox_learning_gain & 0x00ff;
  data[8] = CalcCrc(data+6);
  data[9] = (nox_gating & 0xff00) >> 8;
  data[10] = nox_gating & 0x00ff;
  data[11] = CalcCrc(data+9);
  data[12] = (nox_initial & 0xff00) >> 8;
  data[13] = nox_initial & 0x00ff;
  data[14] = CalcCrc(data+12);
  data[15] = (nox_gain & 0xff00) >> 8;
  data[16] = nox_gain & 0x00ff;
  data[17] = CalcCrc(data+15);

  // Send new value for NOx parameters to sensor (will be hold in RAM, not persistent)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xE1);
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

  // Wait 20 ms to allow the sensor to fill the internal buffer
  delay(20);

  // Send command to read NOx algortihm tuning parameters (0x60E1)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xE1);
  Wire.endTransmission();

  // Wait 20 ms to allow the sensor to fill the internal buffer
  delay(20);

  // Read NOx algorithm tuning parameters from SEN55
  Wire.requestFrom(SEN55_ADDRESS, 18);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // Parse data to make sure that new NOx algorithm tuning parameters are correct
  // offset is arbitrary
  nox_offset = (uint16_t)data[0] << 8 | data[1];
  // learning time is in hours
  nox_learning = (uint16_t)data[3] << 8 | data[4];
  // learning time gain in hours
  nox_learning_gain = (uint16_t)data[6] << 8 | data[7];
  // gating time is in minutes
  nox_gating = (uint16_t)data[9] << 8 | data[10];
  // standard initial is arbitrary
  nox_initial = (uint16_t)data[12] << 8 | data[13];  
  // standard initial is arbitrary
  nox_gain = (uint16_t)data[15] << 8 | data[16];  
 
  // Print new NOx algorithm tuning parameters
  Serial.println("default parameters (offset, learning, learning gain, gating, initial, gain): ");
  Serial.println(nox_offset);
  Serial.println(nox_learning);
  Serial.println(nox_learning_gain);
  Serial.println(nox_gating);
  Serial.println(nox_initial);
  Serial.println(nox_gain);
  Serial.println();

  // Send command to start measurement (0x0021)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();  

  // Wait until command is executed, sensors are ready and fan is initialized
  delay(2000);

  // Output measurement value format
  Serial.println("PM1.0\tPM2.5\tPM4.0\tPM10.0\tVOC_Index\tNOx_Index\tRH\tT");  
}

void loop() {
  uint16_t pm1p0, pm2p5, pm4p0, pm10p0;
  int16_t voc, nox, humidity, temperature;
  uint8_t data[24], counter;
  
  // Send read measurement data command (0x03C4)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x03);
  Wire.write(0xC4);
  Wire.endTransmission();

  // Wait 20 ms for command execution
  delay(20);

  // Read measurement data SEN55, after two bytes a CRC follows
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
