//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_I2C_H_
#define HEADER_FILES_I2C_H_

/* System include statements */


/* Silicon Labs include statements */
#include "em_i2c.h"
#include "stdbool.h"
#include "stdint.h"
#include "em_assert.h"
#include "em_cmu.h"

/* The developer's include statements */
#include "brd_config.h"
#include "sleep_routines.h"
#include "scheduler.h"


//***********************************************************************************
// defined files
//***********************************************************************************


// Application scheduled events




//***********************************************************************************
// global variables
//***********************************************************************************

typedef enum {
  setup_and_write_command_setup,
  register_write,
  prepare_send,
  receive_data,
  write_data,
  stop_state
}DEFINED_STATES;
typedef struct {
bool enable; //Enable I2C peripheral when initialization completed.
bool master; //Set to master (true) or slave (false) mode.
uint32_t refFreq; //I2C reference clock assumed when configuring bus frequency setup.
uint32_t freq; //(Max) I2C bus frequency to use.
I2C_ClockHLR_TypeDef clhr; //Clock low/high ratio control.
uint32_t    sda_pin_route;
uint32_t    scl_pin_route;
bool      sda_en;
bool      scl_en;

} I2C_OPEN_STRUCT;

typedef struct{
  DEFINED_STATES current_state;
  volatile bool busy;
  I2C_TypeDef * peripheral;
  uint32_t device_address;
  uint32_t register_address;
  bool write; //ask about this
  uint32_t * store_result_write_data;
  int transfer_byte_num;
  uint32_t  complete_cb;
} I2C_STATE_MACHINE;

//***********************************************************************************
// Global Functions
//***********************************************************************************
void I2C1_IRQHandler(void);
void I2C0_IRQHandler(void);
void i2c_ack_sm(I2C_STATE_MACHINE *i2c_sm);
bool get_busy(I2C_TypeDef * i2c);
void i2c_rx_valid_sm(I2C_STATE_MACHINE *i2c_sm);
void i2c_stop_sm(I2C_STATE_MACHINE *i2c_sm);
void i2c_open(I2C_TypeDef *i2c_def, I2C_OPEN_STRUCT *i2c_setup);
void i2c_start(uint32_t * result, uint32_t device, uint32_t register_add, bool write_read, int bytes, uint32_t cb, bool reset, I2C_TypeDef * i2c_def);
//***********************************************************************************
// Private Functions
//***********************************************************************************

#endif
