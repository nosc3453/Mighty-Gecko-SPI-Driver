#ifndef BRD_CONFIG_HG
#define BRD_CONFIG_HG

//***********************************************************************************
// Include files
//***********************************************************************************
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"

//***********************************************************************************
// defined files
//***********************************************************************************

// LED 0 pin is
#define LED_RED_PORT       gpioPortD
#define LED_RED_PIN        8
#define LED_RED_DEFAULT    false   // Default false (0) = off, true (1) = on
#define LED_RED_GPIOMODE   gpioModePushPull

// LED 1 pin is
#define LED_GREEN_PORT       gpioPortD
#define LED_GREEN_PIN        9
#define LED_GREEN_DEFAULT    false // Default false (0) = off, true (1) = on
#define LED_GREEN_GPIOMODE   gpioModePushPull

#define MCU_HFXO_FREQ			cmuHFRCOFreq_19M0Hz

// GPIO pin setup
//#define STRONG_DRIVE

#ifdef STRONG_DRIVE
	#define LED_RED_DRIVE_STRENGTH		gpioDriveStrengthStrongAlternateStrong
	#define LED_GREEN_DRIVE_STRENGTH	gpioDriveStrengthStrongAlternateStrong
#else
	#define LED_RED_DRIVE_STRENGTH		gpioDriveStrengthWeakAlternateWeak
	#define LED_GREEN_DRIVE_STRENGTH	gpioDriveStrengthWeakAlternateWeak
#endif

// LETIMER PWM Configuration

#define   PWM_ROUTE_0    LETIMER_ROUTELOC0_OUT0LOC_LOC17
#define   PWM_ROUTE_1    LETIMER_ROUTELOC0_OUT1LOC_LOC16

//Thunderboard RGB LED pin definitions
#define RGB_ENABLE_PORT gpioPortJ
#define RGB_ENABLE_PIN 14
#define RGB0_PORT gpioPortI
#define RGB0_PIN 0
#define RGB1_PORT gpioPortI
#define RGB1_PIN 1
#define RGB2_PORT gpioPortI
#define RGB2_PIN 2
#define RGB3_PORT gpioPortI
#define RGB3_PIN 3
#define RGB_RED_PORT gpioPortD
#define RGB_RED_PIN 11
#define RGB_GREEN_PORT gpioPortD
#define RGB_GREEN_PIN 12
#define RGB_BLUE_PORT gpioPortD
#define RGB_BLUE_PIN 13
#define RGB_DEFAULT_OFF false
#define COLOR_DEFAULT_OFF false
#define RED_RGB_LOC TIMER_ROUTELOC0_CC0LOC_LOC19
#define GREEN_RGB_LOC TIMER_ROUTELOC0_CC1LOC_LOC19
#define BLUE_RGB_LOC TIMER_ROUTELOC0_CC2LOC_LOC19

//Si1133 Routing
#define SI1133_SCL_PORT           gpioPortC
#define SI1133_SCL_PIN            5
#define SI1133_SDA_PORT           gpioPortC
#define SI1133_SDA_PIN            4
#define SI1133_SENSOR_EN_PORT     gpioPortF
#define SI1133_SENSOR_EN_PIN      9
#define SI1133_SCL_EN_DEFAULT     true
#define SI1133_SDA_EN_DEFAULT     true
#define SI1133_SENSOR_EN_DEFAULT  true
#define SENSOR_ENABLE_STRENGTH    gpioDriveStrengthWeakAlternateWeak
#define I2C_ROUTE_SCL_LOC  I2C_ROUTELOC0_SCLLOC_LOC17
#define I2C_ROUTE_SDA_LOC  I2C_ROUTELOC0_SDALOC_LOC17

//UART

#define UART_TX_PORT              gpioPortF
#define UART_TX_PIN               3
#define UART_TX_EN_DEFAULT        1


#define UART_RX_PORT              gpioPortF
#define UART_RX_PIN               4
#define UART_RX_EN_DEFAULT        1

//HM10 defines

#define HM10_LEUART0 LEUART0
#define HM10_BAUDRATE 9600
#define HM10_DATABITS leuartDatabits8
#define HM10_ENABLE leuartEnable
#define HM10_PARITY leuartNoParity
#define HM10_REFFREQ 0
#define HM10_STOPBITS leuartStopbits1
#define LEUART0_TX_ROUTE LEUART_ROUTELOC0_TXLOC_LOC27
#define LEUART0_RX_ROUTE LEUART_ROUTELOC0_RXLOC_LOC27


//SPI defines

#define SPI_PERIPHERAL USART3

#define SPI_ENABLE_EN_DEFAULT 1
#define SPI_ENABLE_PORT   gpioPortF
#define SPI_ENABLE_PIN    8

#define SPI_CS_PORT gpioPortC
#define SPI_CS_PIN  3
#define SPI_CS_EN_DEFAULT 1
#define SPI_CS_ROUTE  USART_ROUTELOC0_CSLOC_LOC18

#define SPI_SCLK_PORT gpioPortC
#define SPI_SCLK_PIN  2
#define SPI_SCLK_EN_DEFAULT 0
#define SPI_SCLK_ROUTE  USART_ROUTELOC0_CLKLOC_LOC18

#define SPI_MOSI_PORT gpioPortC
#define SPI_MOSI_PIN  0
#define SPI_MOSI_EN_DEFAULT 0
#define SPI_MOSI_ROUTE  USART_ROUTELOC0_TXLOC_LOC18

#define SPI_MISO_PORT gpioPortC
#define SPI_MISO_PIN  1
#define SPI_MISO_EN_DEFAULT 0
#define SPI_MISO_ROUTE  USART_ROUTELOC0_RXLOC_LOC18

//Setup for icm_spi_open()

#define ICM_ENABLE usartEnable
#define ICM_REFFREQ 0
#define ICM_BAUDRATE 7000000
#define ICM_DATABITS usartDatabits8
#define ICM_MASTER true
#define ICM_MSBF true
#define ICM_CLOCKMODE usartClockMode3
#define ICM_PRSRXENABLE false
#define ICM_PRSRXCH 0
#define ICM_AUTOTX false
#define ICM_AUTOCSENABLE false
#define ICM_AUTOCSHOLD 0 //need to check these two, we don't want this essentially
#define ICM_AUTOCSSETUP 0
//***********************************************************************************
// function prototypes
//***********************************************************************************

#endif
