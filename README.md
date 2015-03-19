# Set-MPU-9150-Sample-Rate
Setting the MPU-9150 Sampling Rate of the 9 axis and generate an interrupt when the data is ready. 

Microcontroller: TM4C123GH6PM by TI.

Architecture: ARM-Cortex-M4

IDE: Code Composer Studio 6.

Sensor: MPU-9150 by Invensense.

This project is a sample code to Set the rate at which the sensors values from the MPU-9150 are being written to the Data Ready Register. At the rate being set, the IMU sends an interrupt to the controller as DATA_READY. The user inputs the data as decimal value and the code takes care of the rest. 
The function being called from MAIN is the main function forming the skeleton of this demo code and can be used independently in other projects
