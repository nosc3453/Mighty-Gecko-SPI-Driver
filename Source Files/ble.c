/**
 * @file ble.c
 * @author Noah Schwartz
 * @date 11/6/21
 * @brief Contains all the functions to interface the application with the HM-18
 *   BLE module and the LEUART driver
 *
 */


//***********************************************************************************
// Include files
//***********************************************************************************
#include "ble.h"
#include <string.h>

//***********************************************************************************
// defined files
//***********************************************************************************

static char* str;

//***********************************************************************************
// private variables
//***********************************************************************************

/***************************************************************************//**
 * @brief BLE module
 * @details
 *  This module contains all the functions to interface the application layer
 *  with the HM-18 Bluetooth module.  The application does not have the
 *  responsibility of knowing the physical resources required, how to
 *  configure, or interface to the Bluetooth resource including the LEUART
 *  driver that communicates with the HM-18 BLE module.
 *
 ******************************************************************************/

//***********************************************************************************
// Private functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 * Sets up for the leuart operations
 *
 * @details
 * Creates an LEUART_OPEN_STRUCT variable and fills it with information defined in brd_config.h
 *
 *
 * @note
 * Should be called in app_peripheral_setup() after gpio_open(), Si1133_i2c_open(), scheduler_open(), sleep_open(), and led_switch() have been called
 * @param[in]
 *uint32_t tx_event: the event call back for the tx operation
 *
 * @param[in]
 *uint32_t rx_event: the event call back for the rx operation
 *
 ******************************************************************************/

void ble_open(uint32_t tx_event, uint32_t rx_event){

  LEUART_OPEN_STRUCT open_def;
  open_def.enable = HM10_ENABLE;
  open_def.baudrate = HM10_BAUDRATE;
  open_def.databits = HM10_DATABITS;
  open_def.parity = HM10_PARITY;
  open_def.rx_done_evt = rx_event;
  open_def.rx_en = true;
  open_def.rx_loc = LEUART0_RX_ROUTE;
  open_def.rx_pin_en = true;
  open_def.tx_en = true;
  open_def.tx_loc = LEUART0_TX_ROUTE;
  open_def.tx_pin_en = true;
  open_def.tx_done_evt = tx_event;
  open_def.stopbits = HM10_STOPBITS;
  leuart_open(LEUART0, &open_def);
  return;
}


/***************************************************************************//**
 * @brief
 * Starts a ble_write operation
 *
 * @details
 * Calls leuart_start with an input string and an optional length parameter.
 *
 *
 * @note
 *Should be called in scheduled_boot_up_cd() and scheduled_letimer0_uf_cb()
 *
 * @param[in]
 *char *string: input string that will be written
 *
 ******************************************************************************/

void ble_write(char* string){
  str = string;
  uint32_t length = strlen(str);
  leuart_start(LEUART0, str, length);

}

/***************************************************************************//**
 * @brief
 *   BLE Test performs two functions.  First, it is a Test Driven Development
 *   routine to verify that the LEUART is correctly configured to communicate
 *   with the BLE HM-18 module.  Second, the input argument passed to this
 *   function will be written into the BLE module and become the new name
 *   advertised by the module while it is looking to pair.
 *
 * @details
 *   This global function will use polling functions provided by the LEUART
 *   driver for both transmit and receive to validate communications with
 *   the HM-18 BLE module.  For the assignment, the communication with the
 *   BLE module must use low energy design principles of being an interrupt
 *   driven state machine.
 *
 * @note
 *   For this test to run to completion, the phone most not be paired with
 *   the BLE module.  In addition for the name to be stored into the module
 *   a breakpoint must be placed at the end of the test routine and stopped
 *   at this breakpoint while in the debugger for a minimum of 5 seconds.
 *
 * @param[in] *mod_name
 *   The name that will be written to the HM-18 BLE module to identify it
 *   while it is advertising over Bluetooth Low Energy.
 *
 * @return
 *   Returns bool true if successfully passed through the tests in this
 *   function.
 ******************************************************************************/

bool ble_test(char *mod_name){
  uint32_t  str_len;

  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // This test will limit the test to the proper setup of the LEUART
  // peripheral, routing of the signals to the proper pins, pin
  // configuration, and transmit/reception verification.  The test
  // will communicate with the BLE module using polling routines
  // instead of interrupts.
  // How is polling different than using interrupts?
  // ANSWER: With polling, the CPU constantly checks the status of the operation. With interrupts, the device tells the CPU when it needs to do
  //something interrupt related.
  // How does interrupts benefit the system for low energy operation?
  // ANSWER: With interrupts, you can put the system into a low energy mode and it'll wake the system up when it needs to do an interrupt. Overall,
  // energy is saved.
  // How does interrupts benefit the system that has multiple tasks?
  // ANSWER: It can halt each task and focus on one specific task to help it complete.

  // First, you will need to review the DSD HM10 datasheet to determine
  // what the default strings to write data to the BLE module and the
  // expected return statement from the BLE module to test / verify the
  // correct response

  // The test_str is used to tell the BLE module to end a Bluetooth connection
  // such as with your phone.  The ok_str is the result sent from the BLE module
  // to the micro-controller if there was not active BLE connection at the time
  // the break command was sent to the BLE module.
  // Replace the test_str "" with the command to break or end a BLE connection
  // Replace the ok_str "" with the result that will be returned from the BLE
  //   module if there was no BLE connection
  char    test_str[80] = "AT";
  char    ok_str[80] = "OK";


  // output_str will be the string that will program a name to the BLE module.
  // From the DSD HM10 datasheet, what is the command to program a name into
  // the BLE module?
  // The  output_str will be a string concatenation of the DSD HM10 command
  // and the input argument sent to the ble_test() function
  // Replace the output_str "" with the command to change the program name
  // Replace the result_str "" with the first part of the expected result
  // The HM-10 datasheet has an error. This response starts with "OK+Set:"
  //  the backend of the expected response will be concatenated with the
  //  input argument
  char    output_str[80] = "AT+NAME";
  char    result_str[80] = "OK+Set:";


  // To program the name into your module, you must reset the module after you
  // have sent the command to update the modules name.  What is the DSD HM10
  // name to reset the module?
  // Replace the reset_str "" with the command to reset the module
  // Replace the reset_result_str "" with the expected BLE module response to
  //  to the reset command
  char    reset_str[80] = "AT+RESET";
  char    reset_result_str[80] = "OK+RESET";
  char    return_str[80];

  bool    success;
  bool    rx_disabled, rx_en, tx_en;
  uint32_t  status;

  // These are the routines that will build up the entire command and response
  // of programming the name into the BLE module.  Concatenating the command or
  // response with the input argument name
  strcat(output_str, mod_name);
  strcat(result_str, mod_name);

  // The test routine must not alter the function of the configuration of the
  // LEUART driver, but requires certain functionality to insure the logical test
  // of writing and reading to the DSD HM10 module.  The following c-lines of code
  // save the current state of the LEUART driver that will be used later to
  // re-instate the LEUART configuration

  status = leuart_status(HM10_LEUART0);
  if (status & LEUART_STATUS_RXBLOCK) {
    rx_disabled = true;
    // Enabling, unblocking, the receiving of data from the LEUART RX port
    leuart_cmd_write(HM10_LEUART0, LEUART_CMD_RXBLOCKDIS);
  }
  else rx_disabled = false;
  if (status & LEUART_STATUS_RXENS) {
    rx_en = true;
  } else {
    rx_en = false;
    // Enabling the receiving of data from the RX port
    leuart_cmd_write(HM10_LEUART0, LEUART_CMD_RXEN);
  // Why could you be stuck in the below while loop after a write to CMD register?
  // Answer: Waiting until RXEnable is set to true
    while (!(leuart_status(HM10_LEUART0) & LEUART_STATUS_RXENS));
  }

  if (status & LEUART_STATUS_TXENS){
    tx_en = true;
  } else {
    // Enabling the transmission of data to the TX port
    leuart_cmd_write(HM10_LEUART0, LEUART_CMD_TXEN);
    while (!(leuart_status(HM10_LEUART0) & LEUART_STATUS_TXENS));
    tx_en = false;
  }
//  leuart_cmd_write(HM10_LEUART0, (LEUART_CMD_CLEARRX | LEUART_CMD_CLEARTX));

  // This sequence of instructions is sending the break ble connection
  // to the DSD HM10 module.
  // Why is this command required if you want to change the name of the
  // DSD HM10 module?
  // ANSWER: When connected to a device or something similar, you cannot change the name of the module otherwise the things will start ot break down due to
  // devices not knowing what to do.
  str_len = strlen(test_str);
  for (uint32_t i = 0; i < str_len; i++){
    leuart_app_transmit_byte(HM10_LEUART0, test_str[i]);
  }

  // What will the ble module response back to this command if there is
  // a current ble connection?
  // ANSWER: OK
  str_len = strlen(ok_str);
  for (uint32_t i = 0; i < str_len; i++){
    return_str[i] = leuart_app_receive_byte(HM10_LEUART0);
    if (ok_str[i] != return_str[i]) {
        EFM_ASSERT(false);;
    }
  }

  // This sequence of code will be writing or programming the name of
  // the module to the DSD HM10
  str_len = strlen(output_str);
  for (uint32_t i = 0; i < str_len; i++){
    leuart_app_transmit_byte(HM10_LEUART0, output_str[i]);
  }

  // Here will be the check on the response back from the DSD HM10 on the
  // programming of its name
  str_len = strlen(result_str);
  for (uint32_t i = 0; i < str_len; i++){
    return_str[i] = leuart_app_receive_byte(HM10_LEUART0);
    if (result_str[i] != return_str[i]) {
        EFM_ASSERT(false);
    }
  }

  // It is now time to send the command to RESET the DSD HM10 module
  str_len = strlen(reset_str);
  for (uint32_t i = 0; i < str_len; i++){
    leuart_app_transmit_byte(HM10_LEUART0, reset_str[i]);
  }

  // After sending the command to RESET, the DSD HM10 will send a response
  // back to the micro-controller
  str_len = strlen(reset_result_str);
  for (uint32_t i = 0; i < str_len; i++){
    return_str[i] = leuart_app_receive_byte(HM10_LEUART0);
    if (reset_result_str[i] != return_str[i]) {
        EFM_ASSERT(false);;
    }
  }

  // After the test and programming have been completed, the original
  // state of the LEUART must be restored
  if (!rx_en) leuart_cmd_write(HM10_LEUART0, LEUART_CMD_RXDIS);
  if (rx_disabled) leuart_cmd_write(HM10_LEUART0, LEUART_CMD_RXBLOCKEN);
  if (!tx_en) leuart_cmd_write(HM10_LEUART0, LEUART_CMD_TXDIS);
  leuart_if_reset(HM10_LEUART0);

  success = true;


  CORE_EXIT_CRITICAL();
  return success;
}

