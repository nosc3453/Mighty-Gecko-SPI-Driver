/**
 * @file leuart.c
 * @author Noah Schwartz
 * @date 11/6/21
 * @brief Contains all the functions of the LEUART peripheral
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************

//** Standard Library includes
#include <string.h>

//** Silicon Labs include files
#include "em_gpio.h"
#include "em_cmu.h"

//** Developer/user include files
#include "leuart.h"
#include "scheduler.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define ZERO_NUM 0
#define EIGHT 8
//***********************************************************************************
// private variables
//***********************************************************************************
uint32_t  rx_done_evt;
uint32_t  tx_done_evt;
bool    leuart0_tx_busy;
static LEUART_STATE_MACHINE leuart0_machine;


/***************************************************************************//**
 * @brief LEUART driver
 * @details
 *  This module contains all the functions to support the driver's state
 *  machine to transmit a string of data across the LEUART bus.  There are
 *  additional functions to support the Test Driven Development test that
 *  is used to validate the basic set up of the LEUART peripheral.  The
 *  TDD test for this class assumes that the LEUART is connected to the HM-18
 *  BLE module.  These TDD support functions could be used for any TDD test
 *  to validate the correct setup of the LEUART.
 *
 ******************************************************************************/

//***********************************************************************************
// Private functions
//***********************************************************************************



//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 * Initializes various values necessary to do LEUART operations
 *
 * @details
 * This function first enables the clock based on LEUART0. Then, this function checks and clears interrupt flags. Then this function
 * initializes leuart_values based on settings from leuart_settings. Then we call LEUART_Init, passing leuart and leuart_values.
 * Then, we set tx_done_evt and rx_done_evt to what they are in leuart_settings. Lastly, this function enables interrupts.
 *
 *
 * @note
 *Should be called at the end of ble_open()
 *
 * @param[in]
 * LEUART_TypeDef * leuart: leuart peripheral being used
 *
 * @param[in]
 * LEUART_OPEN_STRUCT *leuart_settings: struct that manages the settings and routing settings of the LEUART peripheral.
 ******************************************************************************/

void leuart_open(LEUART_TypeDef *leuart, LEUART_OPEN_STRUCT *leuart_settings){
  if(leuart == LEUART0)
    {
      CMU_ClockEnable(cmuClock_LEUART0, true);
    }
  if ((leuart->IF & LEUART_IF_STARTF) == ZERO_NUM)
    {
      leuart->IFS = LEUART_IF_STARTF;
      EFM_ASSERT(leuart->IF & LEUART_IF_STARTF);
      leuart->IFC = LEUART_IF_STARTF;
    }
  else
  {
      leuart->IFC = LEUART_IF_STARTF;
      EFM_ASSERT(!(leuart->IF & LEUART_IF_STARTF));
  }
  while(leuart->SYNCBUSY & LEUART_SYNCBUSY_STARTFRAME);
  LEUART_Init_TypeDef leuart_values;
  leuart_values.enable = leuartDisable;
  leuart_values.baudrate = leuart_settings->baudrate;
  leuart_values.parity = leuart_settings->parity;
  leuart_values.refFreq = ZERO_NUM;
  leuart_values.databits = leuart_settings->databits;
  leuart_values.stopbits = leuart_settings->stopbits;
  LEUART_Init(leuart, &leuart_values);
  while(leuart->SYNCBUSY);
  leuart->ROUTELOC0 = leuart_settings->rx_loc | leuart_settings->tx_loc;
  leuart->ROUTEPEN = (LEUART_ROUTEPEN_RXPEN * leuart_settings->rx_en) | (LEUART_ROUTEPEN_TXPEN * leuart_settings->tx_en);
  LEUART_Enable(leuart, leuart_settings->enable);
  while(((leuart->STATUS & LEUART_STATUS_RXENS) != LEUART_STATUS_RXENS) && ((leuart->STATUS & LEUART_STATUS_TXENS) != LEUART_STATUS_TXENS));
  EFM_ASSERT((leuart->STATUS & LEUART_STATUS_RXENS) == LEUART_STATUS_RXENS);
  EFM_ASSERT((leuart->STATUS & LEUART_STATUS_TXENS) == LEUART_STATUS_TXENS);
  tx_done_evt = leuart_settings->tx_done_evt;
  rx_done_evt = leuart_settings->rx_done_evt;
//  I2C_IntEnable(leuart, I2C_IEN_TXBL);
//  I2C_IntEnable(leuart, I2C_IEN_TXC);
  NVIC_EnableIRQ(LEUART0_IRQn);

}

/***************************************************************************//**
 * @brief
 *Instructions for the state machine when an TXBL interrupt is received by the master
 *
 * @details
 * When a TXBL interrupt is received, the state will first transfer to the second state. In the second state, one character will be sent and this process
 * will repeat until leuart_sm->count >= leuart_sm->transfer_char_num, in which then the state is changed to the stop_sending_and_finish state and
 * TXBL interrupts are disabled and TXC interrupts are enabled.
 *
 *
 * @note
 *Should be called in LEUART0_IRQHandler() when an TXBL interrupt is scheduled
 *
 * @param[in]
 * LEUART_STATE_MACHINE *leuart_sm: struct that manages the state machine
 ******************************************************************************/
void leuart_txbl_sm(LEUART_STATE_MACHINE * leuart_sm)
{
  switch(leuart_sm->current_state)
  {
  case intro_state:
    leuart_sm->current_state = transmit_data;
    break;
  case transmit_data:
    if(leuart_sm->count < leuart_sm->transfer_char_num)
      {
        leuart_sm->peripheral->TXDATA |= leuart_sm->transmit_array[leuart_sm->count]; // look at this
        leuart_sm->count++;
      }
    else
      {
        leuart_sm->peripheral->IEN &= ~(LEUART_IEN_TXBL);
        leuart_sm->current_state = stop_sending_and_finish;
        leuart_sm->peripheral->IEN |= LEUART_IEN_TXC;
      }
    break;
  case stop_sending_and_finish:
    EFM_ASSERT(false);
    break;
 default:
   EFM_ASSERT(false);
   break;
  }
}
/***************************************************************************//**
 * @brief
 *Instructions for the state machine when an TXC interrupt is received by the master
 *
 * @details
 * When a TXC interrupt is received, TXC interrupts are disabled, the sleep mode is unblocked from EM3, the busy bit is set to false, and tx_done_event
 * is called by add_scheduled_event.
 *
 *
 * @note
 *Should be called in LEUART0_IRQHandler() when an TXC interrupt is scheduled
 *
 * @param[in]
 * LEUART_STATE_MACHINE *leuart_sm: struct that manages the state machine
 ******************************************************************************/
void leuart_txc_sm(LEUART_STATE_MACHINE * leuart_sm)
{
  switch(leuart_sm->current_state)
  {
case intro_state:
  EFM_ASSERT(false);
  break;
case transmit_data:
  EFM_ASSERT(false);
  break;
case stop_sending_and_finish:
  leuart_sm->peripheral->IEN &= ~(LEUART_IEN_TXC);
  sleep_unblock_mode(LEUART_TX_EM);
  leuart_sm->busy = false;
  add_scheduled_event(tx_done_evt);
  break;
default:
 EFM_ASSERT(false);
 break;
  }
}
/***************************************************************************//**
 * @brief
 *Handles the interrupts for LEUART0
 *
 * @details
 *Clears the interrupts from LEUART0, then, depending on the interrupt, a function is called based
 *on that interrupt to handle parts of the write operation.
 *
 * @note
 *Called when an TXBL or an TXC interrupt is being handled
 *
 ******************************************************************************/
void LEUART0_IRQHandler(void){
  uint32_t int_flag;
  int_flag = LEUART0->IF & LEUART0->IEN;
  LEUART0->IFC = int_flag;
  if(int_flag & LEUART_IF_TXBL)
  {
      leuart_txbl_sm(&leuart0_machine);
  }
  if(int_flag & LEUART_IF_TXC)
  {
      leuart_txc_sm(&leuart0_machine);
  }
  return;
}

/***************************************************************************//**
 * @brief
 * Initializes the LEUART_STATE_MACHINE struct for a write operation
 *
 * @details
 * Initializes the LEUART_STATE_MACHINE struct atomically and blocks EM3 as well as enables the TXBL interrupt.
 *
 *
 * @note
 *Should be called in scheduled_boot_up_cd() and scheduled_letimer0_uf_cb()
 *
 * @param[in]
 * LEUART_TypeDef * leuart: leuart peripheral being used
 * @param[in]
 * char *string: input string that will be written
 * @param[in]
 * uint32_t string_len: length of the string being sent over
 ******************************************************************************/

void leuart_start(LEUART_TypeDef *leuart, char *string, uint32_t string_len){
  while(leuart0_machine.busy);
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  leuart0_machine.current_state = intro_state;
  leuart0_machine.busy = true;
  leuart0_machine.transfer_char_num = string_len;
  leuart0_machine.peripheral = leuart;
  leuart0_machine.count = 0;
  for(uint32_t i = 0; i < string_len; i++)
    {
      leuart0_machine.transmit_array[i] = string[i];
    }
  sleep_block_mode(LEUART_TX_EM); //EM3
  leuart0_machine.peripheral->IEN |= LEUART_IEN_TXBL;
  CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 * Checks if LEUART0 in the tx operation is busy.
 *
 * @details
 * Checks if LEUART0 in the tx operation is busy via an if statement. Will be updated to include a check for LEUART1
 *
 *
 * @note
 *Should be called whenever it is necessary to check if the leuart tx operation is busy
 *
 * @param[in]
 * LEUART_TypeDef * leuart: leuart peripheral being used
 *
 * @param[out]
 *leuart0_machine.busy: the status of the LEUART busy variable. If LEUART0 is not in use, this function returns false by default.
 ******************************************************************************/

bool leuart_tx_busy(LEUART_TypeDef *leuart){
  if(leuart == LEUART0)
    {
      return leuart0_machine.busy;
    }
  else
    {
      return false;
    }
return false;
}

/***************************************************************************//**
 * @brief
 *   LEUART STATUS function returns the STATUS of the peripheral for the
 *   TDD test
 *
 * @details
 *   This function enables the LEUART STATUS register to be provided to
 *   a function outside this .c module.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 *   Returns the STATUS register value as an uint32_t value
 *
 ******************************************************************************/

uint32_t leuart_status(LEUART_TypeDef *leuart){
  uint32_t  status_reg;
  status_reg = leuart->STATUS;
  return status_reg;
}

/***************************************************************************//**
 * @brief
 *   LEUART CMD Write sends a command to the CMD register
 *
 * @details
 *   This function is used by the TDD test function to program the LEUART
 *   for the TDD tests.
 *
 * @note
 *   Before exiting this function to update  the CMD register, it must
 *   perform a SYNCBUSY while loop to ensure that the CMD has by synchronized
 *   to the lower frequency LEUART domain.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] cmd_update
 *   The value to write into the CMD register
 *
 ******************************************************************************/

void leuart_cmd_write(LEUART_TypeDef *leuart, uint32_t cmd_update){

  leuart->CMD = cmd_update;
  while(leuart->SYNCBUSY);
}

/***************************************************************************//**
 * @brief
 *   LEUART IF Reset resets all interrupt flag bits that can be cleared
 *   through the Interrupt Flag Clear register
 *
 * @details
 *   This function is used by the TDD test program to clear interrupts before
 *   the TDD tests and to reset the LEUART interrupts before the TDD
 *   exits
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 ******************************************************************************/

void leuart_if_reset(LEUART_TypeDef *leuart){
  leuart->IFC = 0xffffffff;
}

/***************************************************************************//**
 * @brief
 *   LEUART App Transmit Byte transmits a byte for the LEUART TDD test
 *
 * @details
 *   The BLE module will respond to AT commands if the BLE module is not
 *   connected to the phone app.  To validate the minimal functionality
 *   of the LEUART peripheral, write and reads to the LEUART will be
 *   performed by polling and not interrupts.
 *
 * @note
 *   In polling a transmit byte, a while statement checking for the TXBL
 *   bit in the Interrupt Flag register is required before writing the
 *   TXDATA register.
 *
 * @param[in] *leuart
 *   Defines the LEUART peripheral to access.
 *
 * @param[in] data_out
 *   Byte to be transmitted by the LEUART peripheral
 *
 ******************************************************************************/

void leuart_app_transmit_byte(LEUART_TypeDef *leuart, uint8_t data_out){
  while (!(leuart->IF & LEUART_IF_TXBL));
  leuart->TXDATA = data_out;
}


/***************************************************************************//**
 * @brief
 *   LEUART App Receive Byte polls a receive byte for the LEUART TDD test
 *
 * @details
 *   The BLE module will respond to AT commands if the BLE module is not
 *   connected to the phone app.  To validate the minimal functionality
 *   of the LEUART peripheral, write and reads to the LEUART will be
 *   performed by polling and not interrupts.
 *
 * @note
 *   In polling a receive byte, a while statement checking for the RXDATAV
 *   bit in the Interrupt Flag register is required before reading the
 *   RXDATA register.
 *
 * @param[in] leuart
 *   Defines the LEUART peripheral to access.
 *
 * @return
 *   Returns the byte read from the LEUART peripheral
 *
 ******************************************************************************/

uint8_t leuart_app_receive_byte(LEUART_TypeDef *leuart){
  uint8_t leuart_data;
  while (!(leuart->IF & LEUART_IF_RXDATAV));
  leuart_data = leuart->RXDATA;
  return leuart_data;
}
