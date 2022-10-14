# Sample Code for Arduino
Arduino code examples for raw sensor communication with many Sensirion sensors

# Summary
The Arduino Platform allows for easy prototyping with endless possibilities. In addition to the documentation in the datasheet and application notes this repository demonstrates the communication with several Sensirion AG sensors through I2C and UART interfaces. The examples are very basic and typically are a starting point for customer specific implementations. The code for the minimal examples only uses the standard Wire or Serial libraries. No additional external libraries are needed. 

The code for the I2C interface examples is written without the use of abstractions so it could be easily adapted to own projects. To keep the code simple usually no error handling like CRC check or I2C NAK checks are implemented.

The UART examples typically are referring to the SHDLC driver by Sensirion which is included in each example folder. 

# How to use

All samples in this directory share the same format; as such, you can follow the instructions below to get any of them up and running:

1. If you haven't done so yet, install the [Arduino IDE](https://www.arduino.cc/en/software) and run some [basic test programs](https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink) to ensure your system is configured properly
1. Unplug your Arduino from your computer
1. Connect your sensor to the Arduino board. If you're unsure how to do that, please consult the sensors datasheet and the Arduino pinout
1. Reconnect your Arduino to your computer
1. Create a new Sketch in the Arduino IDE
1. Copy the contents of the .ino file and replace the content of your newly created sketch
1. Save your project under a meaningful name
1. Compile and flash the sketch
1. Open the "Serial Monitor" from the "Tools" menu in the Arduino IDE to display the output data read from the sensor

# Examples by sensor

## SCD4x

|Name|Protocol|Description|
|----|--------|-----------|
|[SCD4x_I2C_minimal_example](/SCD4x_I2C_minimal_example)|I2C|Minimal example to get started|
|[SCD4x_I2C_FRC_Forced_Recalibration_Example](SCD4x_I2C_FRC_Forced_Recalibration_Example)|I2C|Show forced recalibration|

## SEN44
|Name|Protocol|Description|
|----|--------|-----------|
|[SEN44_I2C_minimal_example](/SEN44_I2C_minimal_example)|I2C|Basic example for I2C|
|[SEN44_I2C_pm_values_floating_point](/SEN44_I2C_pm_values_floating_point)|I2C|Read out PM values as floating point|
|[SEN44_I2C_change_T_offset_example](/SEN44_I2C_change_T_offset_example)|I2C|Change T offset over I2C|
|[SEN44_I2C_change_VOC_parameters_example](/SEN44_I2C_change_VOC_parameters_example)|I2C|Change VOC parameters over I2C|
|[SEN44_SCD40_I2C_example](/SEN44_SCD40_I2C_example)|I2C|Example using SEN44 with SCD4x|
|[SEN44_UART_minimal_example](/SEN44_UART_minimal_example)|UART|Basic example for UART (Serial) interface|

## SEN5x
|Name|Protocol|Description|
|----|--------|-----------|
|[SEN5x_I2C_minimal_example](/SEN5x_I2C_minimal_example)|I2C|Basic example for I2C|
|[SEN5x_I2C_config_STAR_example](/SEN5x_I2C_config_STAR_example)|I2C|Example configuration of STAR|
|[SEN5x_I2C_config_coldstart_example](/SEN5x_I2C_config_coldstart_example)|I2C|Change T offset for cold start compensation|
|[SEN5x_I2C_config_warmstart_example](/SEN5x_I2C_config_warmstart_example)|I2C|Change T behaviour in warm start scenario|
|[SEN5x_I2C_change_VOC_parameters_example](/SEN5x_I2C_change_VOC_parameters_example)|I2C|Change VOC parameters over I2C|
|[SEN5x_I2C_change_NOx_parameters_example](/SEN5x_I2C_change_NOx_parameters_example)|I2C|Change NOx parameters over I2C|
|[SEN5x_I2C_read_raw](/SEN5x_I2C_read_raw)|I2C|Example for reading raw VOC and NOX values from the sensor over I2C|
|[SEN5x_I2C_switch_measurement_mode](/SEN5x_I2C_switch_measurement_mode)|I2C|Example for switching between gas only and full measurement mode (requires FW2.0) over I2C|


## SFA30
|Name|Protocol|Description|
|----|--------|-----------|
|[SFA30_I2C_minimal_example](/SFA30_I2C_minimal_example)|I2C|Minimal example to get started|


## SGP40
|Name|Protocol|Description|
|----|--------|-----------|
|[SGP40_SHTC3_I2C_voc_algo_minimal_example](/SGP40_SHTC3_I2C_voc_algo_minimal_example)|I2C|Minimal example to use SGP40 with VOC Index algorithm|

## SHT3x
|Name|Protocol|Description|
|----|--------|-----------|
|[SHT3x_I2C_minimal_example](/SHT3x_I2C_minimal_example)|I2C|Minimal example to get started|

## SHT4x
|Name|Protocol|Description|
|----|--------|-----------|
|[SHT4x_I2C_minimal_example](/SHT4x_I2C_minimal_example)|I2C|Minimal example to get started|

## SHTC3
|Name|Protocol|Description|
|----|--------|-----------|
|[SHTC3_I2C_minimal_example](/SHTC3_I2C_minimal_example)|I2C|Minimal example to get started|


## SVM40
|Name|Protocol|Description|
|----|--------|-----------|
|[SVM40_I2C_minimal_example](/SVM40_I2C_minimal_example)|I2C|Minimal example to get started|
|[SVM40_I2C_change_T_offset_example](/SVM40_I2C_change_T_offset_example)|I2C|Change T offset|
|[SVM40_I2C_change_VOC_parameters_example](/SVM40_I2C_change_VOC_parameters_example)|I2C|Change VOC index algorithm parameters|


## SDP8xx
|Name|Protocol|Description|
|----|--------|-----------|
|[SDP8xx_I2C_minimal_example](/SDP8xx_I2C_minimal_example)|I2C|Minimal example to get started|




