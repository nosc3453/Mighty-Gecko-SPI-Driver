//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_SPI_H_
#define HEADER_FILES_SPI_H_


/* System include statements */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Silicon Labs include statements */
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_assert.h"


/* The developer's include statements */
#include "app.h"
#include "brd_config.h"

//***********************************************************************************
// defined files
//***********************************************************************************



//***********************************************************************************
// global variables
//***********************************************************************************
typedef enum {
  spi_intro_state,
  spi_operation_setup,
  spi_read_data,
  spi_write_data,
  spi_stop_state
}SPI_STATES;
typedef struct {
USART_Enable_TypeDef enable;
uint32_t refFreq;
uint32_t baudrate;
USART_Databits_TypeDef  databits;
bool master;
bool msbf;
USART_ClockMode_TypeDef clockMode;
bool prsRxEnable;
USART_PRS_Channel_t prsRxCh;
bool autoTx;
bool autoCsEnable;
uint8_t autoCsHold;
uint8_t autoCsSetup;
bool cs_en;
bool mosi_en;
bool miso_en;
bool sclk_en;

} SPI_OPEN_STRUCT;

typedef struct {
  SPI_STATES current_state;
  uint32_t  cb;
  volatile bool busy;
  USART_TypeDef * peripheral;
  bool operation; //true for read, false for write
  uint32_t reg_add;
  uint32_t * read_val;
  uint32_t write_val;
  int bytes_sent;
  uint32_t bit_bucket;
  bool rx_operation;
} SPI_STATE_MACHINE;


//***********************************************************************************
// function prototypes
//***********************************************************************************
bool spi_busy(USART_TypeDef * usart);
void spi_open(USART_TypeDef *usart, SPI_OPEN_STRUCT * usart_setup);
void spi_start(USART_TypeDef * usart, uint32_t cb, bool operation, uint32_t register_address, uint32_t write_value, uint32_t * read_value, int bytes);
void USART3_TX_IRQHandler(void);
void USART3_RX_IRQHandler(void);
void spi_txbl_sm(SPI_STATE_MACHINE * usart_sm);
void spi_txc_sm(SPI_STATE_MACHINE * usart_sm);
void spi_rxdata_sm(SPI_STATE_MACHINE * usart_sm);


#endif /* HEADER_FILES_SPI_H_ */
