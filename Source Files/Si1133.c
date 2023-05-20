/**
 * @file Si1133.c
 * @author Noah Schwartz
 * @date 10/3/21
 * @brief Set up for the Si1133 as well as calls to the read operation and requesting its result
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "Si1133.h"
#include "app.h"
//***********************************************************************************
// defined files
//***********************************************************************************
#define ZERO_NUM    0
#define ONE_NUM     1
#define SIADD        0x0  //address of the SI1133
#define DADD        0x55 //  address of the Pearl Gecko
#define DELAY   25
#define WRITE_TO_INPUT0 0b000001
#define RESPONSE0 0x11
#define INPUT0  0x0A
#define ADCCONFIG0  0x02
#define COMMAND 0x0B
#define CHAN_LIST 0x01
#define WHITE_COLOR 0x0B
#define PARAM_SET 0b10000000
#define WRITE_CMD_OR_ADC  (PARAM_SET | ADCCONFIG0)
#define WRITE_CMD_OR_CHAN  (PARAM_SET | CHAN_LIST)
#define ZERO_F 0x0F
#define TWO 2
#define FORCE 0x11
#define HOSTOUT0 0x13
//***********************************************************************************
// Private variables
//***********************************************************************************
static uint32_t si1133_read_result;
static uint32_t si1133_write_data;
//***********************************************************************************
// Private functions
//***********************************************************************************

//***********************************************************************************
// Global functions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *Initializes the elements in the I2C_OPEN_STRUCT struct and then calls i2c_open
 *
 * @details
 *Initializes the elements in the I2C_OPEN_STRUCT struct according to their specifications in the
 *datasheet and based on what we need them to be. Once the struct is initialized,
 *i2c_open is called with I2C1 and the address of the I2C_OPEN_STRUCT passed as parameters
 *
 * @note
 *Called when app_peripheral_setup is called
 ******************************************************************************/
void Si1133_i2c_open(void)
{
  I2C_OPEN_STRUCT open_def;
  timer_delay(DELAY);
  open_def.enable = true;
  open_def.master = true;
  open_def.freq = I2C_FREQ_FAST_MAX;
  open_def.refFreq = ZERO_NUM;
  open_def.clhr = i2cClockHLRAsymetric;
  open_def.sda_pin_route = I2C_ROUTE_SCL_LOC;
  open_def.scl_pin_route = I2C_ROUTE_SDA_LOC;
  open_def.sda_en = true;
  open_def.scl_en =  true;
  i2c_open(I2C1, &open_def);
  si1133_configure();
  return;
}
/***************************************************************************//**
 * @brief
 *Begins i2c read operation
 *
 * @details
 *Calls the i2c_start function and passes by the bytes being sent, device address, register address,
 *a true value for reset and and false for write operation, and a callback event due to the reason
 *below, and the I2C1 peripheral as well as an input to read the data.
 *
 *
 * @note
 *Called when either retrieve_result or si1133_configure (called multiple times in this function) is called
 *
 * @param[in]
 *dadda: device address
 *siadda: slave address
 *cb: the callback event
 *bytes: number of bytes to be sent.
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void Si1133_read(uint32_t dadda, uint32_t siadda, uint32_t cb, int byte) //var scope
{
  i2c_start(&si1133_read_result, dadda ,siadda, false, byte, cb, true, I2C1);
}
/***************************************************************************//**
 * @brief
 *Begins i2c write operation
 *
 * @details
 *Calls the i2c_start function and passes by the bytes being sent, device address, register address,
 *a true value for reset and and false for write operation, and a callback event due to the reason
 *below, and the I2C1 peripheral as well as an input to pass the data.
 *
 *
 * @note
 *Called when either Si1133_send_force or si1133_configure (called multiple times in this function) is called
 *
 * @param[in]
 *dadda: device address
 *siadda: slave address
 *cb: the callback event
 *bytes: number of bytes to be sent
 *data: the data being sent
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void Si1133_write(uint32_t dadda, uint32_t siadda, uint32_t cb, uint32_t data, int byte)
{
  si1133_write_data = data;
  i2c_start(&si1133_write_data, dadda ,siadda, true, byte, cb, true, I2C1);
}

/***************************************************************************//**
 * @brief
 *returns si1133_read_result
 *
 * @details
 *returns si1133_read_result, the result of the i2c read operation which is expected to
 *be 51.
 *
 * @note
 *Called when scheduled_si1133_cb is called to check for the result
 *
 *
 *@param[out]
 *returns si1133_read_result, the result of the i2c read operation
 ******************************************************************************/
uint32_t Si1133_pass_result()
{
  return si1133_read_result;
}

/***************************************************************************//**
 * @brief
 *Sends the force command to the Si1133
 *
 * @details
 *Calls Si1133_write with COMMAND as the input register and FORCE as the data to be written.
 *
 * @note
 *Called when scheduled_letimer0_comp1_cb is called
 *
 *
 ******************************************************************************/
void Si1133_send_force()
{
  Si1133_write(DADD, COMMAND, ZERO_NUM, FORCE, ONE_NUM);
  return;
}
/***************************************************************************//**
 * @brief
 *Retrieves the result of the sensing operation
 *
 * @details
 *Calls Si1133_read with HOSTOUT0 as the register and two bytes as the number of bytes read
 *so that accurate light sensing can be made.
 *
 * @note
 *Called when scheduled_letimer0_uf_cb is called
 *
 * @param[in]
 * cb: callback event
 ******************************************************************************/
void retrieve_result(uint32_t cb)
{
  Si1133_read(DADD, HOSTOUT0, cb, TWO);
}
/***************************************************************************//**
 * @brief
 *Configures the SI1133 for a sensing operation.
 *
 * @details
 *Based off of the flow chart, the write and read operations are called numerous times so that the diode
 *is enabled and picks up white light.
 *
 * @note
 *Called at the end of Si1133_i2c_open
 *
 *
 ******************************************************************************/
void si1133_configure()
{
  uint32_t cur_cmd_cnt = 0;
  Si1133_read(DADD, RESPONSE0, ZERO_NUM, ONE_NUM);
  while(get_busy(I2C1));
  cur_cmd_cnt = Si1133_pass_result();
  cur_cmd_cnt = cur_cmd_cnt & ZERO_F; //and with f, obtains current command count and stores it
  si1133_read_result = ZERO_NUM;
  Si1133_write(DADD, INPUT0, ZERO_NUM, WHITE_COLOR, ONE_NUM);
  while(get_busy(I2C1));
  Si1133_write(DADD, COMMAND, ZERO_NUM, WRITE_CMD_OR_ADC, ONE_NUM);
  while(get_busy(I2C1));
  uint32_t new_cmd_cnt = 0;
  Si1133_read(DADD, RESPONSE0, ZERO_NUM, ONE_NUM);
  while(get_busy(I2C1));
  new_cmd_cnt = si1133_read_result;
  new_cmd_cnt = new_cmd_cnt & ZERO_F;
  if(new_cmd_cnt != (cur_cmd_cnt + ONE_NUM))
    {
      EFM_ASSERT(false);
    }
  Si1133_write(DADD, INPUT0, ZERO_NUM, WRITE_TO_INPUT0, ONE_NUM);
  while(get_busy(I2C1));
  Si1133_write(DADD, COMMAND, ZERO_NUM, WRITE_CMD_OR_CHAN, ONE_NUM);
  while(get_busy(I2C1));
  Si1133_read(DADD, RESPONSE0, ZERO_NUM, ONE_NUM);
  while(get_busy(I2C1));
  new_cmd_cnt = Si1133_pass_result();
  new_cmd_cnt = new_cmd_cnt & ZERO_F;
  if(new_cmd_cnt == (cur_cmd_cnt + TWO))
    {
      return;
    }
  else
    {
      EFM_ASSERT(false);
    }
}
