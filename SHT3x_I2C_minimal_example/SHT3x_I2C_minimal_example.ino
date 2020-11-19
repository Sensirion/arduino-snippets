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

// all digital sensors of the SHT3x (SHT3x-DIS) family are sharing the same interface
// SHT3x with I2C address A (Pin 2 connected to GND) = 0x44
// if I2C address B is used (Pin 2 connected to VDD), change address to 0x45
const int16_t SHT_ADDRESS = 0x44;

void setup() {
  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // output format
  Serial.println("RelativeHumidity(percent)\tTemperature(degC)");
  
  // init I2C
  Wire.begin();

  // wait until sensors are ready, < 1 ms according to datasheet
  delay(1);
}

void loop() {
  float temperature, humidity;
  uint8_t data[6], counter;

  // start sht measurement in high repeatability with clock stretching disabled
  Wire.beginTransmission(SHT_ADDRESS);
  Wire.write(0x24);
  Wire.write(0x00);
  Wire.endTransmission();

  // wait for measurement has finished according to datasheet > 15.5 ms
  delay(16);

  // read measurement data sht: 2 bytes T, 1 byte CRC, 2 bytes RH, 1 byte CRC
  Wire.requestFrom(SHT_ADDRESS, 6);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }
  
  // floating point conversion according to datasheet
  // convert T in degC
  temperature = -45 + 175 * (float)((uint16_t)data[0] << 8 | data[1]) / 65535;
  // convert RH in %
  humidity = 100 * (float)((uint16_t)data[3] << 8 | data[4]) / 65535;
  
  Serial.print(humidity);
  Serial.print("\t");
  Serial.print(temperature);
  Serial.println();

  // wait 1 s for next measurement
  delay(1000);
}
