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
  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // wait until sensors startup, > 50 ms according to datasheet
  delay(50);

  // Initiate I2C communication
  Wire.begin();
  
  // Send command to start measurement (0x0021)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();  

  // Wait until sensor is ready, fan is initialized
  delay(1000);

}

void loop() {
  
  uint16_t pm1p0, pm2p5, pm4p0, pm10p0;
  int16_t voc, nox, humidity, temperature;
  uint8_t data[24], counter, cycles;

  // Output format
  Serial.println("PM measurement mode");
  Serial.println("PM1.0\tPM2.5\tPM4.0\tPM10.0\tVOC_Index\tNOx_Index\tRH\tT");

  // Send command to read measurements with PM included (0x0021)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();  

  // Wait until sensor is ready, fan is initialized
  delay(2000);

  // Initialize cycles
  cycles = 0;

  // Read full data for 120s, 60 cycles
  while (cycles<60){

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
  
    // Wait 2s for next measurement
    delay(2000);
    cycles++;
  }

  
  // Send command to switch to gas only measurement mode (0x0037)
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x37);
  Wire.endTransmission();
  delay(2000);
    
  // Output format
  Serial.println("Gas only measurement mode");
  Serial.println("VOC_Index\tNOx_Index\tRH\tT");

  cycles = 0;

  // Read gas only measurement data for 120s, 60 cycles
  while (cycles<60){
    
    // Send read measurement data command (0x03C4)
    Wire.beginTransmission(SEN55_ADDRESS);
    Wire.write(0x03);
    Wire.write(0xC4);
    Wire.endTransmission();

    // Wait 20 ms to allow the sensor to fill the internal buffer
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
  
    Serial.print(String(float(voc) / 10));
    Serial.print("\t\t");
    Serial.print(String(float(nox) / 10));
    Serial.print("\t\t");
    Serial.print(String(float(humidity) / 100));
    Serial.print("\t");
    Serial.print(String(float(temperature) / 200));
    Serial.println();
  
    // Wait 2 s for next measurement
    delay(2000);
    cycles++;
  }

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
