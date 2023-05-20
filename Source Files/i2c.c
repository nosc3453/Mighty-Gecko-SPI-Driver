/**
 * @file i2c.c
 * @author Noah Schwartz
 * @date 10/16/21
 * @brief defined functions that support the i2c operations, including the state machine
 * implementation.
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "i2c.h"


//***********************************************************************************
// defined files
//***********************************************************************************
#define ZERO_NUM    0
#define ONE_BINARY  0x01
#define I2C_EM_BLOCK EM2
#define WRITE_VAL 0x00
#define READ_VAL   0x01
#define EIGHT 8
#define ONE_NUM 1


//***********************************************************************************
// Private variables
//***********************************************************************************

static I2C_STATE_MACHINE i2c0_machine;
static I2C_STATE_MACHINE i2c1_machine;


//***********************************************************************************
// Private functions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *resets I2C state machines so that the read operation stays consistent
 *
 * @details
 *"reset the I2C state machines of the Mighty Gecko I2C peripheral as well as
 * reset the I2C state machines of the external I2C devices such as the Si1133"
 *
 * @note
 *Called in i2c_start when reset is true
 *
 * @param[in]
 *I2C_TypeDef *i2c_def: I2C peripheral being used
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
static void i2c_bus_reset(I2C_TypeDef *i2c_def)
{
  i2c_def->CMD = I2C_CMD_ABORT; //step 1
  uint32_t state_save;
  state_save = i2c_def->IEN; //step 2
  i2c_def->IEN = 0x00; //step 3
  i2c_def->IFC = i2c_def->IF; //step 4
  i2c_def->CMD = I2C_CMD_CLEARTX; //step 5
  i2c_def->CMD = I2C_CMD_START | I2C_CMD_STOP; //step 6
  while(!(i2c_def->IF & I2C_IF_MSTOP)); //step 7
  i2c_def->IFC = i2c_def->IF; //step 8
  i2c_def->CMD = I2C_CMD_ABORT; //step 9
  i2c_def->IEN = state_save; //step 10
  return;
}
/***************************************************************************//**
 * @brief
 *Instructions for the state machine when an ACK interrupt is received by the master
 *
 * @details
 *Defines the instructions for states setup_and_write_command_setup, register_write,
 *and prepare_send whenever an ACK interrupt is called. Updated to also include write operations
 *and a state write_data.
 *
 * @note
 *"responds to the ACK interrupt based on the current state of the state machine"
 *
 * @param[in]
 *I2C_STATE_MACHINE *i2c_sm: struct that manages the state machine
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void i2c_ack_sm(I2C_STATE_MACHINE *i2c_sm)
{
  switch(i2c_sm->current_state)
  {
  case setup_and_write_command_setup:
    if(i2c_sm->write == true) //Write Operation
      {
        i2c_sm->current_state = write_data;
      }
    else if(i2c_sm->write == false)
      {
        i2c_sm->current_state = register_write;
      }
    i2c_sm->peripheral->TXDATA = i2c_sm->register_address;
    break;
  case register_write:
    i2c_sm->current_state = prepare_send;
    i2c_sm->peripheral->CMD = I2C_CMD_START;
    i2c_sm->peripheral->TXDATA = i2c_sm->device_address << ONE_NUM | READ_VAL;
    break;
  case prepare_send:
        i2c_sm->current_state = receive_data;
    break;
  case receive_data:
    EFM_ASSERT(false);
    break;
  case write_data:
    if(i2c_sm->transfer_byte_num != ZERO_NUM)
      {
        i2c_sm->transfer_byte_num--;
        i2c_sm->peripheral->TXDATA |= *(i2c_sm->store_result_write_data) << (EIGHT * i2c_sm->transfer_byte_num); // look at this
        break;
      }
    else
      {
        i2c_sm->peripheral->CMD = I2C_CMD_STOP;
        i2c_sm->current_state = stop_state;
      }
    break;
  case stop_state:
    EFM_ASSERT(false);
    break;
  default:
    EFM_ASSERT(false);
    break;
  }
}
/***************************************************************************//**
 * @brief
 *Instructions for the state machine when an RXDATA VALID interrupt is received by the master
 *
 * @details
 *Defines the instructions for the receive_data state based on when an RXDATA VALID interrupt
 *is sent. Keeps the state the same based on the number of bytes left to be sent, if there
 *are no more bytes to send, state machine transitions into the stop_state
 *
 * @note
 *"responds to the RXDATAV interrupt based on the current state of the state machine"
 *
 * @param[in]
 *I2C_STATE_MACHINE *i2c_sm: struct that manages the state machine
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void i2c_rx_valid_sm(I2C_STATE_MACHINE * i2c_sm)
{
  switch(i2c_sm->current_state)
  {
  case setup_and_write_command_setup:
    EFM_ASSERT(false);
    break;
  case register_write:
    EFM_ASSERT(false);
    break;

  case prepare_send:
    EFM_ASSERT(false);
    break;
  case receive_data:
        *(i2c_sm->store_result_write_data) &= ~(0xFF << (EIGHT * (i2c_sm->transfer_byte_num-1)));
        *(i2c_sm->store_result_write_data) |= i2c_sm->peripheral->RXDATA << (EIGHT * (i2c_sm->transfer_byte_num-1));
        i2c_sm->transfer_byte_num--;
        if(i2c_sm->transfer_byte_num != ZERO_NUM)
          {
            i2c_sm->peripheral->CMD = I2C_CMD_ACK;
          }
        else
          {
            i2c_sm->peripheral->CMD = I2C_CMD_NACK;
            i2c_sm->peripheral->CMD = I2C_CMD_STOP;
            i2c_sm->current_state = stop_state;
          }
    break;
  case write_data:
    EFM_ASSERT(false);
    break;
  case stop_state:
    EFM_ASSERT(false);
    break;
  default:
    EFM_ASSERT(false);
    break;
  }
}
/***************************************************************************//**
 * @brief
 *Instructions for the state machine when an STOP interrupt is sent
 *
 * @details
 *Defines the instructions for stop_state. Unblocks the sleep mode, sets the busy value to
 *false, and adds the scheduled_si1133_cb to be scheduled
 *
 * @note
 *responds to the MSTOP based on the current state of the state machine
 *
 * @param[in]
 *I2C_STATE_MACHINE *i2c_sm: struct that manages the state machine
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void i2c_stop_sm(I2C_STATE_MACHINE *i2c_sm)
{
  switch(i2c_sm->current_state)
  {
  case setup_and_write_command_setup:
    EFM_ASSERT(false);
    break;
  case register_write:
    EFM_ASSERT(false);
    break;
  case prepare_send:
    EFM_ASSERT(false);
    break;
  case receive_data:
    EFM_ASSERT(false);
    break;
  case write_data:
    EFM_ASSERT(false);
    break;
  case stop_state:
    sleep_unblock_mode(I2C_EM_BLOCK);
    i2c_sm->busy = false;
    add_scheduled_event(i2c_sm->complete_cb);
    break;
  default:
    EFM_ASSERT(false);
    break;
  }
}
//***********************************************************************************
// Global functions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *Initializes various values as well as enables interrupts
 *
 * @details
 *First, this function enables the clock based on whether the peripheral being used is I2C0
 *Or I2C1. Then, the function verifies proper I2C clock operation. Then, i2c_values is created
 *and initialized. With i2c_values, i2c_def gets initialized and is also routed.
 *Finally, interrupts are enabled based on whether the peripheral used is I2C0 or I2C1
 *
 * @note
 *Called when at the bottom Si1133_i2c_open
 *
 * @param[in]
 *I2C_TypeDef * i2c_def: I2C peripheral being used
 *I2C_OPEN_STRUCT * i2c_setup: "The STRUCT that the device code will use to define
 *or configure the Mighty Gecko I2C peripheral per the device
 *requirements"
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void i2c_open(I2C_TypeDef *i2c_def, I2C_OPEN_STRUCT *i2c_setup)
{
  if(i2c_def == I2C0)
    {
      CMU_ClockEnable(cmuClock_I2C0, true);
    }
  else if (i2c_def == I2C1)
    {
      CMU_ClockEnable(cmuClock_I2C1, true);
    }
  if ((i2c_def->IF & ONE_BINARY) == ZERO_NUM)
    {
      i2c_def->IFS = ONE_BINARY;
      EFM_ASSERT(i2c_def->IF & ONE_BINARY);
      i2c_def->IFC = ONE_BINARY;
    }
  else
  {
      i2c_def->IFC = ONE_BINARY;
      EFM_ASSERT(!(i2c_def->IF & ONE_BINARY));
  }
  I2C_Init_TypeDef i2c_values;
  i2c_values.enable = i2c_setup->enable;
  i2c_values.master = i2c_setup->master;
  i2c_values.freq = i2c_setup->freq;
  i2c_values.refFreq = i2c_setup->refFreq;
  i2c_values.clhr = i2c_setup->clhr;

  I2C_Init(i2c_def, &i2c_values);
  i2c_def->ROUTELOC0 = i2c_setup->sda_pin_route | i2c_setup->scl_pin_route;
  i2c_def->ROUTEPEN = (I2C_ROUTEPEN_SCLPEN * i2c_setup->scl_en) | (I2C_ROUTEPEN_SDAPEN * i2c_setup->sda_en);
  I2C_IntEnable(i2c_def, I2C_IEN_ACK);
  I2C_IntEnable(i2c_def, I2C_IEN_NACK);
  I2C_IntEnable(i2c_def, I2C_IEN_START);
  I2C_IntEnable(i2c_def, I2C_IEN_MSTOP);
  I2C_IntEnable(i2c_def, I2C_IEN_RXDATAV);
  if(i2c_def == I2C0)
    {
      NVIC_EnableIRQ(I2C0_IRQn);

    }
  else if(i2c_def == I2C1)
    {
      NVIC_EnableIRQ(I2C1_IRQn);
    }

}
/***************************************************************************//**
 * @brief
 *Handles the interrupts for I2C1
 *
 * @details
 *Clears the interrupts from I2C1, then, depending on the interrupt, a function is called based
 *on that interrupt to handle parts of the read operation.
 *
 * @note
 *Called when an ACK, RXDATAV, or MSTOP interrupt is being handled
 *
 ******************************************************************************/
void I2C1_IRQHandler()
{
  uint32_t int_flag;
  int_flag = I2C1->IF & I2C1->IEN;
  I2C1->IFC = int_flag;
  if(int_flag & I2C_IF_ACK)
  {
      i2c_ack_sm(&i2c1_machine);
  }
  if(int_flag & I2C_IF_RXDATAV)
  {
      i2c_rx_valid_sm(&i2c1_machine);
  }

  if(int_flag & I2C_IF_MSTOP)
  {
      i2c_stop_sm(&i2c1_machine);
  }

  return;
}
/***************************************************************************//**
 * @brief
 *Handles the interrupts for I2C0
 *
 * @details
 *Clears the interrupts from I2C0, then, depending on the interrupt, a function is called based
 *on that interrupt to handle parts of the read operation.
 *
 * @note
 *Called when an ACK, RXDATAV, or MSTOP interrupt is being handled
 *
 ******************************************************************************/
void I2C0_IRQHandler(void)
{
  uint32_t int_flag;
  int_flag = I2C0->IF & I2C0->IEN;
  I2C0->IFC = int_flag;
  if(int_flag & I2C_IF_ACK)
  {
      i2c_ack_sm(&i2c0_machine);
  }
  if(int_flag & I2C_IF_RXDATAV)
  {
      i2c_rx_valid_sm(&i2c0_machine);
  }

  if(int_flag & I2C_IF_MSTOP)
  {
      i2c_stop_sm(&i2c0_machine);
  }
  return;
}
/***************************************************************************//**
 * @brief
 *Starts the i2c read operation
 *
 * @details
 *This function creates a I2C_STATE_MACHINE * i2c_sm_pt which points to the address of
 *i2c0_machine if I2C0 is in use or i2c1_machine if I2C1 is in use. Then, while the busy
 *boolean is true, sleep_bloc_mode is called to block EM2. Then, i2c_bus_reset is called if
 *the reset value is true. Then, i2c_sm_pt is initialized followed by a start interrupt to be
 *serviced and a write.
 *
 * @note
 *Called when Si1133_read is called
 *
 * @param[in]
 *uint32_t * result: "Pointer of where to store a read result or get the write data"
 *uint32_t device: I2C device address
 *uint32_t register_add: "I2C register address being requested"
 *bool write_read: currently unused, but indicates whether we are doing a read or write operation.
 *int bytes: The amount of bytes to transfer
 *uint32_t cb: "The callback event to request upon completion of the I2C operation"
 *bool reset: Bool value to denote whether or not I2C0/I2C1 is to be reset.
 *I2C_TypeDef * i2c_def: I2C peripheral being used
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void i2c_start(uint32_t * result, uint32_t device, uint32_t register_add, bool write_read, int bytes, uint32_t cb, bool reset, I2C_TypeDef * i2c_def)
{
  I2C_STATE_MACHINE * i2c_sm_pt;
  if(i2c_def == I2C0)
    {
      i2c_sm_pt = &i2c0_machine;
    }
  if(i2c_def == I2C1)
    {
      i2c_sm_pt = &i2c1_machine;
    }
  while(i2c_sm_pt->busy);
  EFM_ASSERT((I2C1->STATE & _I2C_STATE_STATE_MASK) == I2C_STATE_STATE_IDLE);
  sleep_block_mode(I2C_EM_BLOCK); //EM2
  if(reset == true)
    {
      i2c_bus_reset(i2c_def);
    }
  i2c_sm_pt->current_state = setup_and_write_command_setup;
  i2c_sm_pt->peripheral = i2c_def;
  i2c_sm_pt->busy = true;
  i2c_sm_pt->device_address = device;
  i2c_sm_pt->register_address = register_add;
  i2c_sm_pt->write = write_read;
  i2c_sm_pt->complete_cb = cb;
  i2c_sm_pt->store_result_write_data = result;
  i2c_sm_pt->transfer_byte_num = bytes;
  i2c_sm_pt->peripheral->CMD = I2C_CMD_START;
  i2c_sm_pt->peripheral->TXDATA = i2c_sm_pt->device_address << ONE_NUM | WRITE_VAL;
  return;
}
/***************************************************************************//**
 * @brief
 *Gets the busy value of the related I2C peripheral.
 *
 * @details
 *If i2c is I2C1, returns the busy value of i2c1_machine. If i2c is I2C0, returns the busy value of i2c0_machine.
 *
 * @note
 *Called in si1133_configure whenever a read/write operation is called
 *
 * @param[in]
 *I2C_TypeDef * i2c_def: I2C peripheral being used
 *
 *@param[out]
 *returns the busy value of the I2C_STATE_MACHINE being used.
 ******************************************************************************/
bool get_busy(I2C_TypeDef * i2c)
{
  if(i2c == I2C0)
    {
      return i2c0_machine.busy;
    }
  else if(i2c == I2C1)
    {
      return i2c1_machine.busy;
    }
  EFM_ASSERT(false);
  return false;
}
