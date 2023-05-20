//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_SI1133_H_
#define HEADER_FILES_SI1133_H_

/* System include statements */

/* Silicon Labs include statements */
#include "em_i2c.h"
#include "stdbool.h"
#include "stdint.h"

/* The developer's include statements */

#include "i2c.h"
#include "HW_delay.h"
#include "brd_config.h"

//***********************************************************************************
// defined files
//***********************************************************************************


// Application scheduled events




//***********************************************************************************
// global variables
//***********************************************************************************




//***********************************************************************************
// Global Functions
//***********************************************************************************

void Si1133_i2c_open(void);
void Si1133_read(uint32_t dadda, uint32_t siadda, uint32_t SI1133_LIGHT_READ_CB, int byte);
void Si1133_write(uint32_t dadda, uint32_t siadda, uint32_t SI1133_LIGHT_READ_CB, uint32_t data, int byte);
uint32_t Si1133_pass_result();
void Si1133_send_force();
void retrieve_result(uint32_t cb);
void si1133_configure();

//***********************************************************************************
// Private Functions
//***********************************************************************************


#endif /* HEADER_FILES_SI1133_H_ */
