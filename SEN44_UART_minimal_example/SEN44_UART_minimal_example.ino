#include "sensirion_uart.h"
#include "sen44.h"

// used for sensor measurement return values
struct sen44_measurement m;

char serial[SEN44_MAX_SERIAL_LEN];

void setup() {
  int16_t ret;

  Serial.begin(115200);
  // wait for serial connection from PC
  // comment the following line if you'd like the output
  // without waiting for the interface being ready
  while(!Serial);
  
  // init serial interface to communicate with sensor
  while (sensirion_uart_open() != 0) {
    Serial.println("UART init failed\n");
    // sleep for 1 s
    sensirion_sleep_usec(1000000);
  }
  delay(1000);

  // get sensor serial id
  ret = sen44_get_serial(serial);
 
  Serial.print("SEN44 Serial: ");
  Serial.println(serial);

  // start the measurement mode
  // fan and laser will be activated
  // TVOC and RH,T sensors set to measure mode
  ret = sen44_start_measurement();
  if (ret < 0)
    Serial.println("error starting measurement\n");

  //wait for startup of the fan
  delay(1000);
  
  Serial.println("PM_1.0\tPM_2.5\tPM_4.0\tPM_10.0\tVOCT\tRH\tT");
}

void loop() {
  int16_t ret;
  
  // read sensor data
  ret = sen44_read_measurement(&m);
  if (ret < 0) {
    Serial.println("error reading measurement\n");
    Serial.println(ret);
  }
  
  Serial.print("" + String(m.mc_1p0));      //serial print pm value
  Serial.print("\t" + String(m.mc_2p5));      //serial print pm value
  Serial.print("\t" + String(m.mc_4p0));      //serial print pm value
  Serial.print("\t" + String(m.mc_10p0));      //serial print pm value
  Serial.print("\t" + String(float(m.voc_index)/10));      //serial print NO2 value in ppb
  Serial.print("\t" + String(float(m.ambient_humidity)/100));      //serial print RH value    
  Serial.print("\t" + String(float(m.ambient_temperature)/200));      //serial print T value
  Serial.println();

  delay(1000);
}
