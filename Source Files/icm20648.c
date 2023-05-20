/**
 * @file icm20648.c
 * @author Noah Schwartz
 * @date 11/14/21
 * @brief Configures and allows for the usage of the spi peripheral
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "icm20648.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define CONFIG_DELAY 1
#define OPEN_DELAY  31
#define PWR_MGMT_1_ADDRESS 0x06
#define MGMT1_SET 0x28
#define PWR_MGMT_2_ADDRESS 0x07
#define MGMT2_SET 0x07
#define LP_CONFIG_ADDRESS 0x05
#define LP_CONFIG_SET 0x20
#define ACCEL_WOM_THR_ADDRESS 0x13
#define ACCEL_WOM_THR_SET 0b11110000 >> 2
#define NOEVENT 0
#define TWO_NUM 2
#define ONE_NUM 1
#define ZERO_NUM 0
#define REG_BANK_SEL_ADDRESS 0x7F
#define REG_BANK_SEL_SET1 0x20
#define REG_BANK_SEL_SET2 0x00
#define ACCEL_ZOUT_H 0x31


//***********************************************************************************
// Private variables
//***********************************************************************************
static uint32_t usart_write_data;
static uint32_t usart_read_result;
//***********************************************************************************
// Private functions
//***********************************************************************************

//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// functions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *Initializes the elements in the SPI_OPEN_STRUCT struct and then calls spi_open and icm_configure
 *
 * @details
 *Initializes the elements in the SPI_OPEN_STRUCT struct according to their specifications in the
 *datasheet and based on what we need them to be. Once the struct is initialized,
 *spi_open is called with USART3 and the address of the SPI_OPEN_STRUCT passed as parameters. Then icm_configure is
 *called to initialize the spi peripheral.
 *
 * @note
 *Called when app_peripheral_setup is called
 *
 ******************************************************************************/
void icm_spi_open()
{
  SPI_OPEN_STRUCT open_def;
  timer_delay(OPEN_DELAY);
  open_def.enable = ICM_ENABLE;
  open_def.refFreq = ICM_REFFREQ;
  open_def.baudrate = ICM_BAUDRATE;
  open_def.databits = ICM_DATABITS;
  open_def.master = ICM_MASTER;
  open_def.msbf = ICM_MSBF;
  open_def.clockMode = ICM_CLOCKMODE;
  open_def.prsRxEnable = ICM_PRSRXENABLE;
  open_def.prsRxCh = ICM_PRSRXCH;
  open_def.autoTx = ICM_AUTOTX;
  open_def.autoCsEnable = ICM_AUTOCSENABLE;
  open_def.autoCsHold = ICM_AUTOCSHOLD;
  open_def.autoCsSetup = ICM_AUTOCSSETUP;
  open_def.miso_en = true;
  open_def.mosi_en = true;
  open_def.sclk_en = true;
  open_def.cs_en = false;

  spi_open(USART3, &open_def);
  icm_configure();
}
/***************************************************************************//**
 * @brief
 *Begins spi write operation
 *
 * @details
 *Calls the spi_start function and passes by the bytes being sent, the usart peripheral,
 *the register address, a false value for write operation, a callback event,
 *and a data variable to set usart_write_data to which will be passed when spi_start is called.
 *
 *
 * @note
 * Called whenever icm_configure calls it
 *
 * @param[in]
 *USART_TypeDef * usart: peripheral being used
 *uint32_t cb: the callback event
 *uint32_t register_address: the register address
 *uint32_t data: the data being sent
 *bytes: number of bytes to be sent.
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void icm_spi_write(USART_TypeDef * usart, uint32_t cb, uint32_t register_address, uint32_t data, int byte)
{
  usart_write_data = data;
  spi_start(usart, cb, false, register_address, usart_write_data, ZERO_NUM, byte);
}
/***************************************************************************//**
 * @brief
 *Begins spi read operation
 *
 * @details
 *Calls the spi_start function and passes by the bytes being sent, the usart peripheral,
 *the register address, a true value for the read operation, and a callback event due to the reason
 *below.
 *
 *
 * @note
 * Called whenever icm_configure calls it
 *
 * @param[in]
 *USART_TypeDef * usart: peripheral being used
 *uint32_t cb: the callback event
 *uint32_t register_address: the register address
 *bytes: number of bytes to be sent.
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void icm_spi_read(USART_TypeDef * usart, uint32_t cb, uint32_t register_address, int byte)
{
  spi_start(usart, cb, true, register_address, ZERO_NUM, &usart_read_result, byte);
}
/***************************************************************************//**
 * @brief
 *Begins a read operation to read the z value from the accelerometer
 *
 * @details
 *Calls the spi_start function and passes by two bytes to be read, the usart peripheral,
 *the register address, a true value for the read operation, and a callback event due to the reason
 *below. This function uses ACCEL_ZOUT_H register address to read from.
 *
 *
 * @note
 *Called whenever scheduled_letimer0_uf_cb calls it
 *
 * @param[in]
 *uint32_t cb: the callback event
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void icm_read_print(uint32_t cb)
{
  spi_start(USART3, cb, true, ACCEL_ZOUT_H, ZERO_NUM, &usart_read_result, TWO_NUM);
}
/***************************************************************************//**
 * @brief
 *Returns the value of usart_read_result, the read value of the spi read operation
 *
 * @details
 *Returns the value of usart_read_result, the read value of the spi read operation. This
 *function is necessary since usart_read_result is a static variable and must by passed from
 *a function to reach another .c file.
 *
 *
 * @note
 *Should be called in void scheduled_icm_done_cb()
 *
 * @param[out]
 * returns usart_read_result, the result of a spi reading operation
 ******************************************************************************/
uint32_t icm_get_z_value()
{
  return usart_read_result;
}
/***************************************************************************//**
 * @brief
 *Configures the SPI for usage and also tests whether or not they were configured correctly
 *
 * @details
 *This function covers the configuration for the SPI peripheral and also tests if the configuration
 *is set up correctly. For each call to spi_write and spi_read, we wait for the write/read operation to complete
 *followed by a short delay. For every write in this function, we call read into the same register address
 *and check if the correct values were stored in the register. The first write writes to PWR_MGMT_1 and we
 *write and check it so that low power was enabled and the temperature sensor is disabled. The second write
 *writes to PWR_MGMT_2 and we write and check that the accelerometer is enabled and the gyroscope is disabled
 *The third write writes to LP_CONFIG and we write and check it such that the accelerometer is in duty cycle mode.
 *The final write writes to ACCEL_WOM_THR and we write and check that the accelerometer threshold is 240mg.
 *If the code makes it past this point, the peripheral is configured and the write/read operations work as intended.
 *This code was updated to include writing and reading to the REG_BANK_SEL register.
 * @note
 *Should be called at the end of icm_spi_open()
 *
 ******************************************************************************/
void icm_configure() //THIS FUNCTION IS MY TDD
{
  icm_spi_write(USART3, NOEVENT, PWR_MGMT_1_ADDRESS, MGMT1_SET, ONE_NUM); //enables low power and disables temperature sensor
  while(spi_busy(USART3)); //Need to wait until operation is complete, expected to fail here in the TDD submission. If not, it will fail the coming EFM_ASSERT
  timer_delay(CONFIG_DELAY);
  usart_read_result = ZERO_NUM;
  icm_spi_read(USART3, ZERO_NUM, PWR_MGMT_1_ADDRESS, ONE_NUM);
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  EFM_ASSERT(usart_read_result == MGMT1_SET); //Checks if what was written to spi_write went through correctly. Essentially tests the write and read operations and if they were configured correctly.


  icm_spi_write(USART3, NOEVENT, PWR_MGMT_2_ADDRESS, MGMT2_SET, ONE_NUM); //enables the accelerometer and disables the gyroscope
  while(spi_busy(USART3)); //Need to wait until operation is complete, expected to fail here in the TDD submission. If not, it will fail the coming EFM_ASSERT
  timer_delay(CONFIG_DELAY);
  usart_read_result = ZERO_NUM; //Resets the read result
  icm_spi_read(USART3, ZERO_NUM, PWR_MGMT_2_ADDRESS, ONE_NUM);
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  EFM_ASSERT(usart_read_result == MGMT2_SET);

  icm_spi_write(USART3, NOEVENT, LP_CONFIG_ADDRESS, LP_CONFIG_SET, ONE_NUM); //enables the accelerometer to operate in duty cycle mode
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  usart_read_result = ZERO_NUM;
  icm_spi_read(USART3, ZERO_NUM, LP_CONFIG_ADDRESS, ONE_NUM);
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  EFM_ASSERT(usart_read_result == LP_CONFIG_SET);

  icm_spi_write(USART3, NOEVENT, REG_BANK_SEL_ADDRESS, REG_BANK_SEL_SET1, ONE_NUM); //
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  usart_read_result = ZERO_NUM;
  icm_spi_read(USART3, ZERO_NUM, REG_BANK_SEL_ADDRESS, ONE_NUM);
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  EFM_ASSERT(usart_read_result == REG_BANK_SEL_SET1);


  icm_spi_write(USART3, NOEVENT, ACCEL_WOM_THR_ADDRESS, ACCEL_WOM_THR_SET, ONE_NUM); //Sets the accelerometer threshold to 240mg
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  usart_read_result = ZERO_NUM;
  icm_spi_read(USART3, ZERO_NUM, ACCEL_WOM_THR_ADDRESS, ONE_NUM);
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  EFM_ASSERT(usart_read_result == ACCEL_WOM_THR_SET);


  icm_spi_write(USART3, NOEVENT, REG_BANK_SEL_ADDRESS,REG_BANK_SEL_SET2 , ONE_NUM); //
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  usart_read_result = ZERO_NUM;
  icm_spi_read(USART3, ZERO_NUM, REG_BANK_SEL_ADDRESS, ONE_NUM);
  while(spi_busy(USART3)); //Need to wait until operation is complete
  timer_delay(CONFIG_DELAY);
  EFM_ASSERT(usart_read_result == REG_BANK_SEL_SET2);


  //If function reaches here, the write/read operations work as intended
}

