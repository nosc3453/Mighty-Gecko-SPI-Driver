/**
 * @file spi.c
 * @author Noah Schwartz
 * @date 11/14/21
 * @brief Configuration for the spi operations
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "spi.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define SPI_EM_BLOCK EM2
#define THROWAWAY 0xFF
#define EIGHT 8
#define SEVEN 7
#define ZERO_NUM 0
//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************

static SPI_STATE_MACHINE usart0_machine;
static SPI_STATE_MACHINE usart1_machine;
static SPI_STATE_MACHINE usart2_machine;
static SPI_STATE_MACHINE usart3_machine;


//***********************************************************************************
// Private functions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 * Initializes various values necessary to do SPI operations
 *
 * @details
 * This function first enables the clock based on usart. Then this function
 * initializes spi_values based on settings from usart_setup. Then we call USART_InitSync, passing usart and spi_values.
 * Then, the ROUTELOC0 and ROUTEPEN is set. Finally, TX and RX interrupts are enabled.
 *
 * @note
 *Should be called by the end of icm_spi_open
 *
 * @param[in]
 * USART_TypeDef * usart: spi peripheral being used
 *
 * @param[in]
 * spi_OPEN_STRUCT *usart_setup: struct that manages the settings and routing settings of the spi peripheral.
 ******************************************************************************/
void spi_open(USART_TypeDef *usart, SPI_OPEN_STRUCT * usart_setup)
{
  if(usart == USART0)
    {
      CMU_ClockEnable(cmuClock_USART0, true);
    }
  else if(usart == USART1)
    {
      CMU_ClockEnable(cmuClock_USART1, true);
    }
  else if(usart == USART2)
    {
      CMU_ClockEnable(cmuClock_USART2, true);
    }
  else if(usart == USART3)
    {
      CMU_ClockEnable(cmuClock_USART3, true);
    }
  USART_InitSync_TypeDef spi_values;
  spi_values.enable = usart_setup->enable;
  spi_values.autoCsEnable = usart_setup->autoCsEnable;
  spi_values.master = usart_setup->master;
  spi_values.refFreq = usart_setup->refFreq;
  spi_values.autoCsHold = usart_setup->autoCsHold;
  spi_values.autoCsSetup = usart_setup->autoCsSetup;
  spi_values.autoTx = usart_setup->autoTx;
  spi_values.baudrate = usart_setup->baudrate;
  spi_values.clockMode = usart_setup->clockMode;
  spi_values.databits = usart_setup->databits;
  spi_values.msbf = usart_setup->msbf;
  spi_values.prsRxCh = usart_setup->prsRxCh;
  spi_values.prsRxEnable = usart_setup->prsRxEnable;
  USART_InitSync(usart, &spi_values);
  usart->ROUTELOC0 = SPI_CS_ROUTE | SPI_SCLK_ROUTE | SPI_MOSI_ROUTE | SPI_MISO_ROUTE;
  usart->ROUTEPEN = USART_ROUTEPEN_CSPEN * usart_setup->cs_en;
  usart->ROUTEPEN |= USART_ROUTEPEN_RXPEN * usart_setup->miso_en | USART_ROUTEPEN_TXPEN * usart_setup->mosi_en | USART_ROUTEPEN_CLKPEN * usart_setup->sclk_en;
  USART_IntClear(usart, USART_IFC_TXC);

  NVIC_EnableIRQ(USART3_TX_IRQn);
  NVIC_EnableIRQ(USART3_RX_IRQn);
}
/***************************************************************************//**
 * @brief
 *Instructions for the state machine when an TXBL interrupt is received by the master
 *
 * @details
 * This function handles the spi read and write operations for when a TXBL interrupt is called.
 * In the intro state, a write to the TXDATA is done and the current state is moved to the spi_operation_setup
 * state. If the operation is a read, rx_operation is set to true. In spi_operation_setup, it first checks
 * if the operation is a read and if it is, sets the rx_operation to true if it isn't. If the
 * operation is a write the first byte of data to be written is saved and the state is moved to spi_write_data.
 * If the state is spi_read_data, a throwaway value is sent and the rx_operation is set to true if the rx_operation is false
 * beforehand. In spi_write_data, one byte of data is sent and if there are no more bytes to send,
 * TXBL interrupts are disabled while TXC interrupts are enabled and the current state is moved
 * to spi_stop_state
 *
 *
 * @note
 *Should be called in USART3_TX_IRQHandler() or USART3_RX_IRQHandler when an TXC interrupt
 *is scheduled
 *
 * @param[in]
 * USART_STATE_MACHINE *usart_sm: struct that manages the state machine
 ******************************************************************************/
void spi_txbl_sm(SPI_STATE_MACHINE * usart_sm)
{
  switch(usart_sm->current_state)
  {
  case spi_intro_state:
    usart_sm->peripheral->TXDATA = (usart_sm->operation << SEVEN) | usart_sm->reg_add; //send txdata from spi peripheral to controller
    usart_sm->current_state = spi_operation_setup; //set state to the next state
    if(usart_sm->operation == true) //this is a read case
    {
        usart_sm->rx_operation = true;
    }
    break;
  case spi_operation_setup:
    if(usart_sm->operation == true) //read
      {
        if(usart_sm->rx_operation != true)
          {
            usart_sm->rx_operation = true;
          }
      }
    else
      {
        if(usart_sm->bytes_sent > ZERO_NUM)
          {
            usart_sm->bytes_sent--;
            usart_sm->peripheral->TXDATA = usart_sm->write_val << (EIGHT * usart_sm->bytes_sent);
          }
        usart_sm->current_state = spi_write_data;
      }
    break;
  case spi_read_data:
    if(usart_sm->rx_operation == false)
      {
        usart_sm->peripheral->TXDATA = THROWAWAY;
        usart_sm->rx_operation = true;
      }
    break;
  case spi_write_data:
    if(usart_sm->bytes_sent > ZERO_NUM)
      {
        usart_sm->bytes_sent--;
        usart_sm->peripheral->TXDATA = usart_sm->write_val << (EIGHT * usart_sm->bytes_sent);
      }
        if(usart_sm->bytes_sent == ZERO_NUM)
          {
            USART_IntDisable(usart_sm->peripheral, USART_IEN_TXBL);
            USART_IntEnable(usart_sm->peripheral, USART_IEN_TXC);
            usart_sm->current_state = spi_stop_state;
          }
    break;
  case spi_stop_state:
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
 * When a TXC interrupt is received, TXC interrupts are disabled, the sleep mode is unblocked from EM3,
 * the busy bit is set to false, the CS pin is driven high, the current_state is set to
 * spi_intro_state and usart_sm->cb is called by add_scheduled_event.
 *
 *
 * @note
 *Should be called in USART3_TX_IRQHandler() or USART3_RX_IRQHandler when an TXC interrupt
 *is scheduled
 *
 * @param[in]
 * USART_STATE_MACHINE *usart_sm: struct that manages the state machine
 ******************************************************************************/
void spi_txc_sm(SPI_STATE_MACHINE * usart_sm)
{
  switch(usart_sm->current_state)
  {
  case spi_intro_state:
    EFM_ASSERT(false);
    break;
  case spi_operation_setup:
    EFM_ASSERT(false);
    break;
  case spi_read_data:
    EFM_ASSERT(false);
    break;
  case spi_write_data:
    EFM_ASSERT(false);
    break;
  case spi_stop_state:
    USART_IntDisable(usart_sm->peripheral, USART_IEN_TXC);
    usart_sm->current_state = spi_intro_state;
    add_scheduled_event(usart_sm->cb);
    usart_sm->busy = false;
    GPIO_PinOutSet(SPI_CS_PORT,SPI_CS_PIN);
    sleep_unblock_mode(SPI_EM_BLOCK);
    break;
  default:
    EFM_ASSERT(false);
    break;
  }
}
/***************************************************************************//**
 * @brief
 *Instructions for the state machine when an RXDATAV interrupt is received by the master
 *
 * @details
 * This function checks to make sure that it is time for an RXDATAV interrupt with the
 * rx_operation variable. This is necessary as a TXBL interrupt can happen at the same time,
 * which can disrupt the flow of the operation. If it is time for a RXDATAV interrupt, then
 * the state machine will progress. In spi_operation_setup, data is sent over to the bit bucket
 * and the current state is changed to spi_read_data. If spi_read_data is the current state, a byte
 * is read and if there's no more bytes to be sent, TXBL and RXDATAV interrupts are disabled
 * and TXC interrupts are enabled. The current_state is moved to the spi_stop_state.
 *
 *
 * @note
 *Should be called in USART3_RX_IRQHandler when an TXC interrupt is scheduled
 *
 * @param[in]
 * USART_STATE_MACHINE *usart_sm: struct that manages the state machine
 ******************************************************************************/
void spi_rxdata_sm(SPI_STATE_MACHINE * usart_sm)
{
  switch(usart_sm->current_state)
  {
  case spi_intro_state:
    EFM_ASSERT(false);
    break;
  case spi_operation_setup:
    if(usart_sm->rx_operation == true)
      {
        usart_sm->bit_bucket = usart_sm->peripheral->RXDATA;
        usart_sm->current_state = spi_read_data;
        usart_sm->rx_operation = false;
      }
    break;
  case spi_read_data:
    if(usart_sm->rx_operation == true)
      {
    if(usart_sm->bytes_sent != ZERO_NUM)
      {
        usart_sm->bytes_sent--;
        *(usart_sm->read_val) &= ~(THROWAWAY << (EIGHT * usart_sm->bytes_sent));
        *(usart_sm->read_val) |= usart_sm->peripheral->RXDATA << (EIGHT * usart_sm->bytes_sent);
        if(usart_sm->bytes_sent == ZERO_NUM)
          {
            USART_IntDisable(usart_sm->peripheral, USART_IEN_TXBL);
            USART_IntDisable(usart_sm->peripheral, USART_IEN_RXDATAV);
            USART_IntEnable(usart_sm->peripheral, USART_IEN_TXC);
            usart_sm->current_state = spi_stop_state;
          }
      }
    usart_sm->rx_operation = false;
      }
    break;
  case spi_write_data:
    EFM_ASSERT(false);//    usart_sm->bit_bucket = usart_sm->peripheral->RXDATA;
   // usart_sm->current_state = spi_read_data;
    break;
  case spi_stop_state:
    EFM_ASSERT(false);
    break;
  default:
    EFM_ASSERT(false);
    break;
  }
}
/***************************************************************************//**
 * @brief
 *Handles the TX type interrupts for USART3
 *
 * @details
 *Clears the interrupts from USART3, then, depending on the interrupt, a function is called based
 *on that interrupt to handle parts of the read operation.
 *
 * @note
 *Called when an TXBL or TXC interrupt is being handled
 *
 ******************************************************************************/
void USART3_TX_IRQHandler()
{
  uint32_t int_flag;
  int_flag = USART3->IF & USART3->IEN;
  USART3->IFC = int_flag;
  if(int_flag & USART_IF_TXBL)
  {
      spi_txbl_sm(&usart3_machine);
  }
  if(int_flag & USART_IF_TXC)
  {
      spi_txc_sm(&usart3_machine);
  }

  return;
}
/***************************************************************************//**
 * @brief
 *Handles the RX type interrupts for USART3
 *
 * @details
 *Clears the interrupts from USART3, then, depending on the interrupt, a function is called based
 *on that interrupt to handle parts of the read operation.
 *
 * @note
 *Called when an TXBL, TXC, or RXDATAV interrupt is being handled
 *
 ******************************************************************************/
void USART3_RX_IRQHandler()
{
  uint32_t int_flag;
  int_flag = USART3->IF & USART3->IEN;
  USART3->IFC = int_flag;
  if(int_flag & USART_IF_TXBL)
  {
      spi_txbl_sm(&usart3_machine);
  }
  if(int_flag & USART_IF_TXC)
  {
      spi_txc_sm(&usart3_machine);
  }
  if(int_flag & USART_IF_RXDATAV)
  {
      spi_rxdata_sm(&usart3_machine);
  }

  return;
}
/***************************************************************************//**
 * @brief
 *Starts a spi read/write operation
 *
 * @details
 * Initializes the SPI_STATE_MACHINE struct atomically and based on
 * what spi peripheral is being used. blocks EM2 as well as enables
 * the TXBL interrupt and RXDATAV interrupts if the operation is a read operation.

 *
 *
 * @note
 * Called whenever icm_configure calls it
 *
 * @param[in]
 *USART_TypeDef * usart: peripheral being used
 *uint32_t cb: the callback event
 *bool operation: a bool value that denotes the operation, true is for read, false is for write.
 *uint32_t register_address: the register address
 *uint32_t write_value: the value that is to be written.
 *uint32_t * read_value: the value that will save the value read.
 *bytes: number of bytes to be sent.
 ******************************************************************************/
void spi_start(USART_TypeDef * usart, uint32_t cb, bool operation, uint32_t register_address, uint32_t write_value, uint32_t * read_value, int bytes)
{
  SPI_STATE_MACHINE * spi_sm_pt;
  if(usart == USART0)
    {
      spi_sm_pt = &usart0_machine;
    }
  if(usart == USART1)
    {
      spi_sm_pt = &usart1_machine;
    }
  if(usart == USART2)
    {
      spi_sm_pt = &usart2_machine;
    }
  if(usart == USART3)
    {
      spi_sm_pt = &usart3_machine;
    }
  while(spi_sm_pt->busy);
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  spi_sm_pt->peripheral->CMD = USART_CMD_CLEARTX;
  spi_sm_pt->peripheral->CMD = USART_CMD_CLEARRX;
  spi_sm_pt->peripheral = usart;
  spi_sm_pt->busy = true;
  spi_sm_pt->cb = cb;
  spi_sm_pt->current_state = spi_intro_state;
  spi_sm_pt->operation = operation; //true for read, false for write
  spi_sm_pt->reg_add = register_address;
  spi_sm_pt->write_val = write_value;
  spi_sm_pt->read_val = read_value;
  spi_sm_pt->bytes_sent = bytes;
  sleep_block_mode(SPI_EM_BLOCK);
  GPIO_PinOutClear(SPI_CS_PORT,SPI_CS_PIN);
  if(operation == true)
    {
      USART_IntEnable(spi_sm_pt->peripheral, USART_IEN_RXDATAV);
    }
  spi_sm_pt->rx_operation = false;
  USART_IntEnable(spi_sm_pt->peripheral, USART_IEN_TXBL);
  CORE_EXIT_CRITICAL();
  return;
}
/***************************************************************************//**
 * @brief
 *Checks if the spi peripheral being used is busy
 *
 * @details
 *Checks the peripheral being used and, depending on what peripheral is being used, returns
 *the busy value of that peripheral.
 *
 *
 * @note
 * Called whenever icm_configure calls it
 *
 * @param[in]
 *USART_TypeDef * usart: peripheral being used
 *
 *@param[out]
 *bool busy: the busy value of the peripheral being used.
 ******************************************************************************/
bool spi_busy(USART_TypeDef * usart)
{
  if(usart == USART0)
    {
      return usart0_machine.busy;
    }
  else if(usart == USART1)
    {
      return usart1_machine.busy;
    }
  else if(usart == USART2)
    {
      return usart2_machine.busy;
    }
  else if(usart == USART3)
    {
      return usart3_machine.busy;
    }
  EFM_ASSERT(false);
  return false;
}
