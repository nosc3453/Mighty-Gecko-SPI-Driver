//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_ICM20648_H_
#define HEADER_FILES_ICM20648_H_

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
#include "spi.h"

//***********************************************************************************
// defined files
//***********************************************************************************



//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void icm_spi_open(void);
void icm_configure(void);
void icm_spi_write(USART_TypeDef * usart, uint32_t cb, uint32_t register_address, uint32_t data, int byte);
void icm_spi_read(USART_TypeDef * usart, uint32_t cb, uint32_t register_address, int byte);
void icm_read_print(uint32_t cb);
uint32_t icm_get_z_value();
#endif /* HEADER_FILES_ICM20648_H_ */
