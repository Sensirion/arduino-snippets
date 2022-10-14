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

// SEN55
const int16_t SEN55_ADDRESS = 0x69;

void setup() {
  Serial.begin(115200);
  // Wait for serial connection from PC
  // Comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // Output format
  Serial.println("VOC_Index\tNOx_Index\tRH\tT");
  
  // wait until sensors startup, > 50 ms according to datasheet
  delay(50);

  // init I2C
  Wire.begin();
  

  // start up sensor, sensor will go to continuous measurement mode
  // each second there will be new measurement values
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();

  // wait until sensor is ready, fan is initialized
  delay(1000);
}

void loop() {

  int16_t voc, nox, humidity, temperature;
  uint8_t data[12], counter;

  // send read measurement data command (0x03D2)l
  Wire.beginTransmission(SEN55_ADDRESS);
  Wire.write(0x03);
  Wire.write(0xD2);
  Wire.endTransmission();
  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // read measurement data SEN44, after two bytes a CRC follows
  Wire.requestFrom(SEN55_ADDRESS, 12);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // VOC level is a signed int 
  // humidity is a signed int and scaled by 100 and need to be divided by 100
  // temperature is a signed int and scaled by 200 and need to be divided by 200
  humidity = (uint16_t)data[0] << 8 | data[1];
  temperature  = (uint16_t)data[3] << 8 | data[4];
  voc = (uint16_t)data[6] << 8 | data[7];
  nox = (uint16_t)data[9] << 8 | data[10];

  Serial.print(String(int(voc)));
  Serial.print("\t");
  Serial.print(String(int(nox)));
  Serial.print("\t");
  Serial.print(String(float(humidity) / 100));
  Serial.print("\t");
  Serial.print(String(float(temperature) / 200));
  Serial.println();

  // wait 1 s for next measurement
  delay(1000);
}
