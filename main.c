#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"


#define MPU9150_I2C_ADDRESS     0x68

tI2CMInstance g_sI2CInst; // Global instance structure for the I2C master driver.
tMPU9150 g_sMPU9150Inst; // Global instance structure for the ISL29023 sensor driver.

volatile uint_fast8_t g_vui8I2CDoneFlag; // Global flags to alert main that MPU9150 I2C transaction is complete
volatile uint_fast8_t g_vui8ErrorFlag; // Global flags to alert main that MPU9150 I2C transaction error has occurred.
volatile uint_fast8_t g_vui8DataFlag; // Global flags to alert main that MPU9150 data is ready to be retrieved.


//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


//*****************************************************************************
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//*****************************************************************************
void MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    if(ui8Status == I2CM_STATUS_SUCCESS)
        g_vui8I2CDoneFlag = 1;

    // Store the most recent status in case it was an error condition
    g_vui8ErrorFlag = ui8Status;
}


//*****************************************************************************
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//*****************************************************************************
void IntGPIOb(void)
{
    unsigned long ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    // Clear all the pin interrupts that are set
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
        // MPU9150 Data is ready for retrieval and processing.
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
}


//*****************************************************************************
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//*****************************************************************************
void MPU9150I2CIntHandler(void)
{
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    I2CMIntHandler(&g_sI2CInst);
}


//*****************************************************************************
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//*****************************************************************************
void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    while(1);
}


//*****************************************************************************
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//*****************************************************************************
void MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0));

    // If an error occurred call the error handler immediately.
    if(g_vui8ErrorFlag)
        MPU9150AppErrorHandler(pcFilename, ui32Line);

    // clear the data flag for next use.
    g_vui8I2CDoneFlag = 0;
}


//*****************************************************************************
// Configure the I2C and its pins.  This better be called before MPU9512
//*****************************************************************************
void ConfigureI2C(void)
{
	// The I2C3 peripheral must be enabled before use.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
}


//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureMPU9150Interrupt(void)
{
	// Configure and Enable the GPIO interrupt. Used for INT signal from the MPU9150
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	ROM_IntEnable(INT_GPIOB);
}


//*****************************************************************************
void SetUpPeripheralsInterrupts(void)
{
	// Keep only some parts of the systems running while in sleep mode.
	// GPIOB is for the MPU9150 interrupt pin.
	// I2C3 is the I2C interface to the ISL29023
	ROM_SysCtlPeripheralClockGating(true);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);

	// Enable interrupts to the processor.
	ROM_IntMasterEnable();
}


//*****************************************************************************
// Initialize all sensors inputs such as filters and settings, very specific
// to the appliation, rather than the generic initalization in the Configure fcn
//*****************************************************************************
void InitializeMPU9150(void)
{
	// Initialize the MPU9150 Driver.
	MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS, MPU9150AppCallback, &g_sMPU9150Inst);
	// Wait for transaction to complete
	MPU9150AppI2CWait(__FILE__, __LINE__);

	// Write application specifice sensor configuration such as filter settings and sensor range settings.
	g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_260_256; // ?? Hz acc/gyro bandwidth
	g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_2000; // Gyro max range of +-2000 dps
	g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ | MPU9150_ACCEL_CONFIG_AFS_SEL_16G); // High Pass filter of 5Hz and max range for +-16g for acc
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);

	// Configure the data ready interrupt pin output of the MPU9150.
	g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL | MPU9150_INT_PIN_CFG_INT_RD_CLEAR | MPU9150_INT_PIN_CFG_LATCH_INT_EN;
	g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG, g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);

    // Modifying the ranges of the sensors
    // look up MPU9150Init function in mpu9150.C if you wanna do this configuration during the initialization
    MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_ACCEL_CONFIG, ~MPU9150_ACCEL_CONFIG_AFS_SEL_M, MPU9150_ACCEL_CONFIG_AFS_SEL_16G, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
	MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M, MPU9150_GYRO_CONFIG_FS_SEL_2000, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
}


//*****************************************************************************
// Set the Sampling Rate of the sensors to the registers
//*****************************************************************************
void SetMpuSampleRate(int16_t sampleRate)
{
	// Read Current Gyro sampling rate, either 8k or 1k by checking the Low pass filter status
	uint8_t dataRead;
	MPU9150Read(&g_sMPU9150Inst, MPU9150_O_CONFIG, &dataRead, 1, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);

	// isolate to get the last 3 bits to get the LPF status
	dataRead |= 0b111;

	// check if rate == 8k or 1k Hz, which corresponds to (1 or 7 for 8k) and others for 1 k
	// calculate the sampling rate using the formula provided in the data sheet
	// Sample Rate = Gyroscope Output Rate / (1 + sample rate divider)
	uint8_t sampleRateDivider;
	if (dataRead == 0 || dataRead == 7)
		sampleRateDivider = (8000 / sampleRate) - 1;
	else
		sampleRateDivider = (1000 / sampleRate) - 1;

	// write to the sample rate divider register
	MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_SMPLRT_DIV, 0, sampleRateDivider, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
}



//*****************************************************************************
// Main application entry point.
//*****************************************************************************
int main(void)
{

    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable port B used for motion interrupt.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    ConfigureI2C();
    ConfigureMPU9150Interrupt();
    SetUpPeripheralsInterrupts();

    // Initialize I2C3 peripheral.
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, ROM_SysCtlClockGet());
    InitializeMPU9150();

    // Set the interrupt Rate, the main reason of this sample code
    SetMpuSampleRate(200);

	while(1);
}


