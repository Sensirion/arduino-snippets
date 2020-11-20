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

// SDP810 500 Pa
const int16_t SDP810_ADDRESS = 0x25;

void setup() {
  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // output format
  Serial.println("DP\tT");
  
  // init I2C
  Wire.begin();
  
  // wait until sensors startup, > 25 ms according to datasheet
  delay(25);

  // start up sensor, sensor will go to continuous measurement mode
  // differential pressure mode with averaging till read 
  Wire.beginTransmission(SDP810_ADDRESS);
  Wire.write(0x36);
  Wire.write(0x15);
  Wire.endTransmission();

  // wait until first data is ready, > 8 ms
  delay(10);
}

void loop() {

  uint16_t scaling;
  int16_t dp, temperature;
  uint8_t data[9], counter;

  // read measurement data, after two bytes a CRC follows
  Wire.requestFrom(SDP810_ADDRESS, 9);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  dp = (uint16_t)data[0] << 8 | data[1];
  temperature = (uint16_t)data[3] << 8 | data[4];
  scaling = (uint16_t)data[6] << 8 | data[7];


  Serial.print(String(float(dp) / scaling));
  Serial.print("\t");
  Serial.print(String(float(temperature) / 200));
  Serial.println();

  // wait 100 ms for next measurement
  delay(100);
}
