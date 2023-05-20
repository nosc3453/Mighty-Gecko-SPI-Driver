/**
 * @file gpio.c
 * @author Noah Schwartz
 * @date 9/9/21
 * @brief Sets up the GPIO for incoming and outgoing signals
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *Sets up related peripherals to the GPIO
 *
 * @details
 *Sets up the clock and LED signals (peripherals) to be sent to and from the GPIO, enabling them to be controlled
 *
 * @note
 *Should be configured after the clock tree is configured
 *
 ******************************************************************************/

void gpio_open(void){

  CMU_ClockEnable(cmuClock_GPIO, true);

	// Configure LED pins
	GPIO_DriveStrengthSet(LED_RED_PORT, LED_RED_DRIVE_STRENGTH);
	GPIO_PinModeSet(LED_RED_PORT, LED_RED_PIN, LED_RED_GPIOMODE, LED_RED_DEFAULT);

	GPIO_DriveStrengthSet(LED_GREEN_PORT, LED_GREEN_DRIVE_STRENGTH);
	//Sensor Enabling
	GPIO_PinModeSet(SI1133_SENSOR_EN_PORT, SI1133_SENSOR_EN_PIN, gpioModePushPull, SI1133_SENSOR_EN_DEFAULT);
	GPIO_PinModeSet(SI1133_SCL_PORT, SI1133_SCL_PIN, gpioModePushPull, SI1133_SCL_EN_DEFAULT);
	GPIO_PinModeSet(SI1133_SDA_PORT, SI1133_SDA_PIN, gpioModePushPull, SI1133_SDA_EN_DEFAULT);

	//LEUART Enabling
  GPIO_DriveStrengthSet(UART_TX_PORT, gpioDriveStrengthStrongAlternateWeak);
  GPIO_PinModeSet(UART_TX_PORT, UART_TX_PIN, gpioModePushPull, UART_TX_EN_DEFAULT);
  GPIO_PinModeSet(UART_RX_PORT, UART_RX_PIN, gpioModeInput, UART_RX_EN_DEFAULT);


	GPIO_PinModeSet(LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_GPIOMODE, LED_GREEN_DEFAULT);
	// Set RGB LED gpio pin configurations
	 GPIO_PinModeSet(RGB_ENABLE_PORT, RGB_ENABLE_PIN, gpioModePushPull,
	RGB_DEFAULT_OFF);
	 GPIO_PinModeSet(RGB0_PORT, RGB0_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	 GPIO_PinModeSet(RGB1_PORT, RGB1_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	 GPIO_PinModeSet(RGB2_PORT, RGB2_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	 GPIO_PinModeSet(RGB3_PORT, RGB3_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
	 GPIO_PinModeSet(RGB_RED_PORT, RGB_RED_PIN, gpioModePushPull, COLOR_DEFAULT_OFF);
	 GPIO_PinModeSet(RGB_GREEN_PORT, RGB_GREEN_PIN, gpioModePushPull,
	COLOR_DEFAULT_OFF);
	 GPIO_PinModeSet(RGB_BLUE_PORT, RGB_BLUE_PIN, gpioModePushPull, COLOR_DEFAULT_OFF);

	 //SPI Set up
   GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, SPI_CS_EN_DEFAULT);
   GPIO_PinModeSet(SPI_SCLK_PORT, SPI_SCLK_PIN, gpioModePushPull, SPI_SCLK_EN_DEFAULT);
   GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, SPI_MOSI_EN_DEFAULT); //TX OPERATION
   GPIO_PinModeSet(SPI_MISO_PORT, SPI_MISO_PIN, gpioModeInput, SPI_MISO_EN_DEFAULT); //RX OPERATION
   GPIO_PinModeSet(SPI_ENABLE_PORT, SPI_ENABLE_PIN, gpioModePushPull, SPI_ENABLE_EN_DEFAULT);

   GPIO_DriveStrengthSet(SPI_MOSI_PORT, gpioDriveStrengthStrongAlternateWeak);



}
