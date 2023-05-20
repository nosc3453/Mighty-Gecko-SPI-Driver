//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef APP_HG
#define APP_HG

/* System include statements */
#include <stdio.h>

/* Silicon Labs include statements */
#include "em_cmu.h"
#include "em_assert.h"

/* The developer's include statements */
#include "cmu.h"
#include "gpio.h"
#include "letimer.h"
#include "brd_config.h"
#include "i2c.h"
#include "Si1133.h"
#include "ble.h"
#include "HW_delay.h"
#include "icm20648.h"
#include "spi.h"


//***********************************************************************************
// defined files
//***********************************************************************************
#define   PWM_PER         2.0   // PWM period in seconds
#define   PWM_ACT_PER     0.002  // PWM active period in seconds

// Application scheduled events
#define LETIMER0_COMP0_CB 0x00000001 //0b0001
#define LETIMER0_COMP1_CB 0x00000002 //0b0010
#define LETIMER0_UF_CB 0x00000004 //0b0100
#define SI1133_LIGHT_READ_CB 0x00000008 //0b1000
#define BOOT_UP_CB 0x00000010 //0b10000
#define TX_EVENT_CB 0x00000020
#define RX_EVENT_CB 0x00000040
#define BLE_TX_DONE_CB 0x00000080
#define ICM_DONE_CB 0x00000100

//EM
#define SYSTEM_BLOCK_EM   EM3


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void app_peripheral_setup(void);
void scheduled_letimer0_uf_cb(void);
void scheduled_letimer0_comp1_cb(void);
void scheduled_letimer0_comp0_cb(void);
void led_switch(void);
void scheduled_si1133_cb(void);
void scheduled_boot_up_cd(void);
void scheduled_ble_tx_done_cd(void);
void scheduled_icm_done_cb(void);
#endif
