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

// SCD40
const int16_t SCD_ADDRESS = 0x62;

void setup() {
  float co2, temperature, humidity;
  uint16_t calibration;
  uint8_t data[12], counter, repetition;
  uint8_t ret;
  
  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);
  
  // init I2C
  Wire.begin();

  // wait until sensors are ready, > 1000 ms according to datasheet
  delay(1000);
  
  // start scd measurement in periodic mode, will update every 2 s
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();

  // wait for first measurement to be finished
  delay(2000);

  Serial.println("# CO2 values before recalibration");

  // measure 5 times 
  for(repetition = 0; repetition < 5; repetition++) {
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
    temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
    // convert RH in %
    humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;
  
    Serial.print("# ");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print(humidity);
    Serial.println();

    delay(2000);
  }
  
  // wait for another 5 minutes to equilibrate sensor to ambient
  Serial.println("# Waiting 5 minutes for equilibration");
  //delay(5 * 60 * 1000);

  // stop scd measurement
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x3f);
  Wire.write(0x86);
  ret = Wire.endTransmission();
  Serial.println(ret);


  // wait for sensor
  //delay(20);

  // assuming an external reference shows 350 ppm
  calibration = 650;
  
  // prepare buffer with data for calibration
  // calculate CRC for each 2 bytes of data
  data[0] = (calibration & 0xff00) >> 8;
  data[1] = calibration & 0x00ff;
  data[2] = CalcCrc(data);
  
  // send command for perform_forced_recalibration
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x36);
  Wire.write(0x2F);
  // append data for calibration
  // 2 bytes calibraion, CRC
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  ret = Wire.endTransmission();
  Serial.println(ret);

  delay(400);
  
  // read data: 2 bytes correction, 1 byte CRC
  Wire.requestFrom(SCD_ADDRESS, 3);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  if(CalcCrc(data) != data[2])
    Serial.println("# ERROR: recalibration CRC return value");
  
  calibration = ((uint16_t)data[0] << 8 | data[1]);
  
  Serial.print("# Value after recalibration\n# ");
  Serial.println(calibration-32768);

  // output format
  Serial.println("CO2(ppm)\tTemperature(degC)\tRelativeHumidity(percent)");

  // start scd measurement again in periodic mode, will update every 2 s
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();

  // wait for first measurement to be finished
  delay(4000);
}

void loop() {
  float co2, temperature, humidity;
  uint8_t data[12], counter;

  // send read data command
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
  temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
  // convert RH in %
  humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

  Serial.print(co2);
  Serial.print("\t");
  Serial.print(temperature);
  Serial.print("\t");
  Serial.print(humidity);
  Serial.println();

  // wait 2 s for next measurement
  delay(2000);
}

// calculate CRC according to datasheet section 5.17
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
