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

// SFA30
const int16_t SFA_ADDRESS = 0x5D;

void setup() {
  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // init I2C
  Wire.begin();

  // wait until sensor is ready
  delay(10);
  
  // start SFA measurement in periodic mode, will update every 0.5 s
  Wire.beginTransmission(SFA_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x06);
  Wire.endTransmission();

  // module is not outputing HCHO for the first 10 s after powering up
  delay(10000);

}

void loop() {
  float hcho, temperature, humidity;
  uint8_t data[9], counter;

  // send read data command
  Wire.beginTransmission(SFA_ADDRESS);
  Wire.write(0x03);
  Wire.write(0x27);
  Wire.endTransmission();

  //wait time before reading for the values should be more than 2ms
  delay(10);
  
  // read measurement data: 
  // 2 bytes formaldehyde, 1 byte CRC, scale factor 5
  // 2 bytes RH, 1 byte CRC, scale factor 100
  // 2 bytes T, 1 byte CRC, scale factor 200
  // stop reading after 9 bytes (not used)
  Wire.requestFrom(SFA_ADDRESS, 9);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }
  
  // floating point conversion according to datasheet
  hcho = (float)((int16_t)data[0] << 8 | data[1])/5;
  // convert RH in %
  humidity = (float)((int16_t)data[3] << 8 | data[4])/100;
  // convert T in degC
  temperature = (float)((int16_t)data[6] << 8 | data[7])/200;

  Serial.print(hcho);
  Serial.print("\t");
  Serial.print(temperature);
  Serial.print("\t");
  Serial.print(humidity);
  Serial.println();

  // wait 1 s for next measurement read out, sensor is averaging in between
  delay(1000);


}
