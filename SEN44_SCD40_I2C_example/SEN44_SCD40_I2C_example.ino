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
// SCD40
const int16_t SCD_ADDRESS = 0x62;

void setup() {
  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // output format
  Serial.println("CO2\tRH_SCD\tT_SCD\tPM1.0\tPM2.5\tPM4.0\tPM10.0\tVOC_Index\tRH\tT");
  
  // init I2C
  Wire.begin();
  
  // wait until sensors startup, > 1 ms according to datasheet
  delay(1);

  // start scd measurement in periodic mode, will update every 2 s
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();
  
  // start up sensor, sensor will go to continous measurement mode
  // each second there will be new measurement values
  Wire.beginTransmission(SEN44_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x21);
  Wire.endTransmission();

  // wait until sensors is ready, fan is initialized
  delay(2000);
}

void loop() {
  uint16_t pm1p0, pm2p5, pm4p0, pm10p0;
  int16_t voc, humidity, temperature;
  float co2, temp_scd, rh_scd;
  uint8_t data[21], counter;

  // read measurement data SCD
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0xec);
  Wire.write(0x05);
  Wire.endTransmission();

  // read measurement data: 2 bytes co2, 1 byte CRC,
  // 2 bytes T, 1 byte CRC, 2 bytes RH, 1 byte CRC,
  // 2 bytes sensor status, 1 byte CRC
  // stop reading after 12 bytes (not used)
  // other data like  ASC not included
  Wire.requestFrom(SCD_ADDRESS, 12);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // floating point conversion according to datasheet
  co2 = (float)((uint16_t)data[0] << 8 | data[1]);
  // convert T in degC
  temp_scd = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
  // convert RH in %
  rh_scd = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

  Serial.print(co2);
  Serial.print("\t");
  Serial.print(rh_scd);
  Serial.print("\t");
  Serial.print(temp_scd);
  Serial.print("\t");
  
  // send read measurement data command
  Wire.beginTransmission(SEN44_ADDRESS);
  Wire.write(0x03);
  Wire.write(0x74);
  Wire.endTransmission();
  
  // wait 10 ms to allow the sensor to fill the internal buffer
  delay(10);

  // read measurement data SEN44, after two bytes a CRC follows
  Wire.requestFrom(SEN44_ADDRESS, 21);
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
  voc = (uint16_t)data[12] << 8 | data[13];
  humidity = (uint16_t)data[15] << 8 | data[16];
  temperature = (uint16_t)data[18] << 8 | data[19];

  Serial.print(pm1p0);
  Serial.print("\t");
  Serial.print(pm2p5);
  Serial.print("\t");
  Serial.print(pm4p0);
  Serial.print("\t");
  Serial.print(pm10p0);
  Serial.print("\t");
  Serial.print(String(float(voc) / 10));
  Serial.print("\t");
  Serial.print(String(float(humidity) / 100));
  Serial.print("\t");
  Serial.print(String(float(temperature) / 200));
  Serial.println();

  // wait 2 s for next measurement, due to sampling frequency of SCD
  delay(2000);
}
