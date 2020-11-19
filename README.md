# Sample Code for Arduino
Arduino code examples for raw sensor communication with many Sensirion sensors

# Summary
The Arduino Platform allows for easy prototyping with endless possibilities. In addition to the documentation in the datasheet and application notes this repository demonstrates the communication with several Sensirion AG sensors through I2C and UART interfaces. The examples are very basic and typically are a starting point for customer specific implementations. The code for the minimal examples only uses the standard Wire or Serial libraries. No additional external libraries are needed. 

The code for the I2C interface examples is written without the use of abstractions so it could be easily adapted to own projects. To keep the code simple usually no error handling like CRC check or I2C NAK checks are implemented.

The UART examples typically are referring to the SHDLC driver by Sensirion which is included in each example folder. 

