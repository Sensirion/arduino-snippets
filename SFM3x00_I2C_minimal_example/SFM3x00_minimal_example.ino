/*
 * Copyright (c) 2021, Sensirion AG
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

// supported sensors
enum SFM_MODEL {
  SFM3000 = 0,
  SFM3200,
  SFM3300,
  SFM3400,
  SFM_MODEL_LENGTH //< Note: this is not a valid value for 'MODEL' below
};

// from the datasheets:
const int16_t SFM3X00_ADDRESS                    =  0x40;

const int16_t FLOW_OFFSET[SFM_MODEL_LENGTH]      = { 32000, 32768, 32768, 32768 };
const int16_t SCALE_FACTOR_AIR[SFM_MODEL_LENGTH] = {   140,   140,   120,   800 };

const byte    CMD_START_MEASUREMENT[]            = { 0x10, 0x00 }; 
const byte    CMD_SOFT_RESET[]                   = { 0x20, 0x00 }; 


// ACTION: select your component here from the enum above:
const uint8_t MODEL = SFM3200;

void setup() 
{
  Serial.begin(115200);
  while(!Serial);
  
  Wire.begin();

  /*
  // optional reset before use
  Wire.beginTransmission(SFM3X00_ADDRESS);
  Wire.write(CMD_SOFT_RESET, 2);
  Wire.endTransmission();
  delay(100);
  */

  Wire.beginTransmission(SFM3X00_ADDRESS);
  Wire.write(CMD_START_MEASUREMENT, 2);
  Wire.endTransmission();

  delay(100);
  Serial.println("Flow rate [slm]");
}

void loop() 
{
  Wire.requestFrom(SFM3X00_ADDRESS, 3);
  if (Wire.available() < 3) {
    Serial.println("No data received from sensor");
  } else {
    int16_t flow;
    flow  = (int16_t)Wire.read() << 8;
    flow |= Wire.read();
    // CRC verification (third byte) left as an exercise for the reader

    flow = (flow - FLOW_OFFSET[MODEL]) / SCALE_FACTOR_AIR[MODEL];
    Serial.println(flow);
  }

  delay(100);
}