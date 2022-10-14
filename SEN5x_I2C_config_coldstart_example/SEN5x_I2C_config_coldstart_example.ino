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
  int16_t t_offset, t_slope, t_time;
  uint8_t data[9], counter;

  Serial.begin(115200);
  
  // Wait for serial connection from PC
  // Comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // Wait until sensors startup, > 50 ms according to datasheet
  delay(50);

  // Initiate I2C communication
  Wire.begin();

  // Send command to read/write temperature compensation parameters (0x60B2)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xB2);
  Wire.endTransmission();

  // Wait for 20 ms to allow the sensor to fill the internal buffer
  delay(20);

  // Read preset temperature compensation parameters from SEN55
  Wire.requestFrom(SEN55_ADDRESS, 9);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // Processing preset t offset, t slope and t time data respectively
  // CRC byte (every 3rd byte) is excluded from processing
  t_offset = (uint16_t)data[0] << 8 | data[1];
  t_slope = (uint16_t)data[3] << 8 | data[4];
  t_time = (uint16_t)data[6] << 8 | data[7];

  // Print t values (divided by scale factor)
  Serial.println();
  Serial.print("Preset T Offset: ");
  Serial.print(String(float(t_offset)/200));
  Serial.println();

  Serial.print("Preset Slope: ");
  Serial.print(String(float(t_slope)/10000));
  Serial.println();

  Serial.print("Preset Time constant: ");
  Serial.print(String(float(t_time)/1));
  Serial.println();

  // Set new values for temperature compensation parameters
  // Set t offset to new value (e.g. -5 degC) and scale it accordingly by a factor of 200
  t_offset = -5 * 200;
  // Set t slope to new value (e.g. 0.01 degC/degC) and scale it accordingly by a factor of 10000
  t_slope = 0.01 * 10000;
  // Set t time constant to new value (in seconds)
  t_time = 600;

  // Parsing 
  data[0] = (t_offset & 0xff00) >> 8;
  data[1] = t_offset & 0x00ff;
  data[2] = CalcCrc(data);
  data[3] = (t_slope & 0xff00) >> 8;
  data[4] = t_slope & 0x00ff;
  data[5] = CalcCrc(data+3);
  data[6] = (t_time & 0xff00) >> 8;
  data[7] = t_time & 0x00ff;
  data[8] = CalcCrc(data+6);

  // Send new temperature compensation parameters to sensor (will be held in RAM, not persistent)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xB2);
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
  Wire.write(data[4]);
  Wire.write(data[5]);
  Wire.write(data[6]);
  Wire.write(data[7]);
  Wire.write(data[8]);
  Wire.endTransmission();

  // Wait 20 ms to allow the sensor to fill the internal buffer
  delay(20);

 // Send command to read new temperature compensation parameters (for confirmation)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x60);
  Wire.write(0xB2);
  Wire.endTransmission();

  // Wait 20 ms to allow the sensor to fill the internal buffer
  delay(20);

  // Read temperature compensation parameters from SEN55
  Wire.requestFrom(SEN55_ADDRESS, 9);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // Parse data to make sure that new temperature compensation parameters are correct
  t_offset = (uint16_t)data[0] << 8 | data[1];
  t_slope = (uint16_t)data[3] << 8 | data[4];
  t_time = (uint16_t)data[6] << 8 | data[7];

  // Print new temperature compensation parameters
  Serial.println();
  Serial.print("New T Offset: ");
  Serial.print(String(float(t_offset)/200));
  Serial.println();

  Serial.print("New Slope: ");
  Serial.print(String(float(t_slope)/10000));
  Serial.println();

  Serial.print("New Time Constant: ");
  Serial.print(String(float(t_time)/1));
  Serial.println();
  
  // Send command to start measurement (0x0021)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();  

  // Wait until sensors is ready, fan is initialized
  delay(2000);

  // Output measurement value format
  Serial.println();
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
  
  // Wait 20 ms to allow the sensor to fill the internal buffer
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
