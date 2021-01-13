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
#include "sensirion_voc_algorithm.h"

// SGP4x
const int16_t SGP_ADDRESS = 0x59;
// SHTC3
const int16_t SHT_ADDRESS = 0x70;

VocAlgorithmParams voc_algorithm_params;

void setup() {
  Serial.begin(115200);  // start serial for output
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);

  // output format
  Serial.println("VOC(level)\tVOC_raw(a.u.)\tRelativeHumidity(percent)\tTemperature(degC)");
  
  // init I2C
  Wire.begin();

  // init VOC engine
  VocAlgorithm_init(&voc_algorithm_params);

  // wait until sensors are ready, < 1 ms according to datasheet
  delay(1);
}

void loop() {
  int32_t voc_raw, voc_index, temperature, humidity;
  uint16_t rh_sgp, t_sgp;
  uint8_t data[6], counter;
  
  // start sht measurement in normal mode with clock stretching disabled
  Wire.beginTransmission(SHT_ADDRESS);
  Wire.write(0x78);
  Wire.write(0x66);
  Wire.endTransmission();

  // wait for measurement has finished according to datasheet > 12.1 ms
  delay(13);

  // read measurement data sht: 2 bytes T, 1 byte CRC, 2 bytes RH, 1 byte CRC
  Wire.requestFrom(SHT_ADDRESS, 6);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }

  // floating point conversion according to datasheet
  // convert T
  // float temperature;
  // temperature = -45 + 175 * (float)((uint16_t)data_receive[0]<<8 | data_receive[1])/65536;
  // convert RH
  // float humidity;
  // humidity = 100 * (float)((uint16_t)data_receive[3]<<8 | data_receive[4])/65536;
 
  /**
  * formulas for conversion of the sensor signals, optimized for fixed point
  * algebra: Temperature = 175 * S_T / 2^16 - 45
  * Relative Humidity = * 100 * S_RH / 2^16
  * https://github.com/Sensirion/embedded-sht/blob/master/sht3x/sht3x.c
  * result is in 1/1000 deg C and 1/1000 % RH
  */
  temperature = (21875 * ((int32_t)data[0] << 8 | data[1]) >> 13) - 45000;
  humidity = (12500 * ((int32_t)data[3] << 8 | data[4]) >> 13);  

  // calculate rh, t for voc on chip compensation;
  // this is basically the reverse conversion
  // to get the raw data from the rht sensor,
  // another option is to directly use the 
  // rht sensor output
  t_sgp = (temperature / 1000 + 45) * 65536 / 175;
  rh_sgp = (humidity / 1000) * 65536 / 100;

  // prepare buffer with rht compensation data
  // calculate CRC for each 2 bytes of data
  data[0] = (rh_sgp & 0xff00) >> 8;
  data[1] = rh_sgp & 0x00ff;
  data[2] = CalcCrc(data);
  data[3] = (t_sgp & 0xff00) >> 8;
  data[4] = t_sgp & 0x00ff;
  data[5] = CalcCrc(data + 3);
  
  // start sgp measurement in mode sgp40_measure_raw
  Wire.beginTransmission(SGP_ADDRESS);
  Wire.write(0x26);
  Wire.write(0x0F);
  // append data for rh and t for compensation
  // as in chapter 4.7 of the datasheet
  // 2 bytes rh, CRC, 2 bytes t, CRC
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
  Wire.write(data[4]);
  Wire.write(data[5]);
  Wire.endTransmission();

  // wait for measurement has finished according to datasheet > 30 ms
  delay(30);

  //read measurement data sgp: 2 bytes voc raw signal, CRC
  Wire.requestFrom(SGP_ADDRESS, 3);
  counter = 0;
  while (Wire.available()) {
    data[counter++] = Wire.read();
  }
  
  // convert 2 bytes to one word
  voc_raw = (int16_t)data[0] << 8 | data[1];
  // convert raw signal to voc index
  VocAlgorithm_process(&voc_algorithm_params, voc_raw, &voc_index);

  Serial.print(voc_raw);
  Serial.print("\t");
  Serial.print(voc_index);
  Serial.print("\t");
  Serial.print(humidity);
  Serial.print("\t");
  Serial.print(temperature);
  Serial.println();

  // wait 1 s for next measurement
  delay(1000);
}

// calculate CRC according to datasheet section 4.6
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
