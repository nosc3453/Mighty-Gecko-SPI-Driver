/**
 * @file app.c
 * @author Noah Schwartz
 * @date 9/23/21
 * @brief Defines for functions that set up the peripherals used
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "app.h"
#include "LEDs_thunderboard.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define EXPECTED_VALUE 51
#define ZERO_NUM    0
#define ONE_NUM     1
#define TWO_NUM     2
#define SIADD        0x0  //address of the SI1133
#define DADD        0x55 //  address of the Pearl Gecko
#define EXPECTED_VALUE_TWENTY 20
//#define BLE_TEST_ENABLED
//***********************************************************************************
// Private variables
//***********************************************************************************
static int var_hold;
static uint32_t x = 3;
static uint32_t y = 0;
static bool isFlipped = false;


//***********************************************************************************
// Private functions
//***********************************************************************************

static void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route);

void scheduled_letimer0_uf_cb(void);
//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *Starts the associated peripherals, updated to initialize scheduler, sleep, I2C, LEUART, and led routines.
 *
 * @details
 *Starts the clock, GPIO, and LETIMER0, sleep, scheduler, LEDs, LEUART operation as well as configures the LETIMER0 peripheral for PWM operation
 *
 *
 * @note
 *Should be called when the clock tree and GPIO have been configured
 *
 ******************************************************************************/

void app_peripheral_setup(void){
  cmu_open();

  gpio_open();
  Si1133_i2c_open();
  scheduler_open();
  sleep_open();
  led_switch();
  icm_spi_open();
  ble_open(TX_EVENT_CB,RX_EVENT_CB); //need to change later!
  sleep_block_mode(SYSTEM_BLOCK_EM);
  app_letimer_pwm_open(PWM_PER, PWM_ACT_PER, PWM_ROUTE_0, PWM_ROUTE_1);
  add_scheduled_event(BOOT_UP_CB); //does this go here?
}

/***************************************************************************//**
 * @brief
 *"Initializes LETIMER0 for PWM operation, updated to include cb and irq enable values"
 *
 * @details
 *"Initializing LETIMER0 for PWM operation by creating the
 *letimer_pwm_struct and initializing all of its elements
 *APP_LETIMER_PWM_TypeDef is defined in letimer.h"
 *
 * @note
 *Called when app_peripheral_setup is called
 *
 * @param[in]
 *period: PWM frequency
 *act_period: how much time the LEDs are active
 *out0_route: Routing for the red LED
 *out1_route: Routing for the green LED
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/




void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route){
  // Initializing LETIMER0 for PWM operation by creating the
  // letimer_pwm_struct and initializing all of its elements
  // APP_LETIMER_PWM_TypeDef is defined in letimer.h
    APP_LETIMER_PWM_TypeDef letimer_pwm_struct;


    letimer_pwm_struct.debugRun = false;     // True = keep LETIMER running will halted
    letimer_pwm_struct.enable = false; //set to false for i2c
    letimer_pwm_struct.out_pin_route0 = out0_route;   // out 0 route to gpio port/pin
    letimer_pwm_struct.out_pin_route1 = out1_route;   // out 1 route to gpio port/pin
    letimer_pwm_struct.out_pin_0_en = false;   // enable out 0 route, set to false for i2c
    letimer_pwm_struct.out_pin_1_en = false;   // enable out 1 route, set to false for i2c
    letimer_pwm_struct.period = period;       // seconds
    letimer_pwm_struct.active_period = act_period;    // seconds
    letimer_pwm_struct.comp0_irq_enable = false;
    letimer_pwm_struct.comp0_cb = LETIMER0_COMP0_CB;
    letimer_pwm_struct.comp1_irq_enable = true;
    letimer_pwm_struct.comp1_cb = LETIMER0_COMP1_CB;
    letimer_pwm_struct.uf_irq_enable = true;
    letimer_pwm_struct.uf_cb = LETIMER0_UF_CB;

    //EFM_ASSERT(false);

letimer_pwm_open(LETIMER0, &letimer_pwm_struct);

}
/***************************************************************************//**
 * @brief
 * Schedules the LED to change into a different color.
 *
 * @details
 *Currently gets the result from the sensing operation. Updated to do a ble_write() operation and a spi read operation.
 *
 *
 * @note
 *Should be called whenever it is called from the IRQ handler.
 *
 ******************************************************************************/
void scheduled_letimer0_uf_cb(void)
{
//  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_UF_CB));
//  if(var_hold == 0)
//    {
//      leds_enabled(RGB_LED_0, COLOR_RED , false);
//      var_hold++;
//    }
//  else if(var_hold == 1)
//    {
//      leds_enabled(RGB_LED_0, COLOR_GREEN , false);
//      var_hold++;
//    }
//  else if(var_hold == 2)
//    {
//      leds_enabled(RGB_LED_0, COLOR_BLUE, false);
//      var_hold = 0;
//    }
  retrieve_result(SI1133_LIGHT_READ_CB);
  //x = x+3;
  //y = y+1;
  //float z = (float) x/y;
  char str[14];
  uint16_t z_val = 0;
  z_val = icm_get_z_value();
  short acc_z = 0;
  acc_z = (short) z_val;
  int final_z = 0;
  final_z = (int) acc_z;
  sprintf(str, "z = %3.1d\n", final_z);
  ble_write(str);
  icm_read_print(ICM_DONE_CB);
  while(spi_busy(USART3));
}
/***************************************************************************//**
 * @brief
 * The COMP0 callback event
 *
 * @details
 *Currently does nothing.
 *
 *
 * @note
 *Should be called whenever it is called from the IRQ handler.
 *
 ******************************************************************************/
void scheduled_letimer0_comp0_cb(void)
{
//  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP0_CB));
}
/***************************************************************************//**
 * @brief
 * Changes the LED to a different color.
 *
 * @details
 *Sends the force command to the COMMAND register
 *
 *
 * @note
 *Should be called whenever it is called from the IRQ handler.
 *
 ******************************************************************************/
void scheduled_letimer0_comp1_cb(void)
{
//  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP1_CB));
//  if(var_hold == 0)
//    {
//      leds_enabled(RGB_LED_0, COLOR_RED, true);
//    }
//  else if(var_hold == 1)
//    {
//      leds_enabled(RGB_LED_0, COLOR_GREEN, true);
//    }
//  else if(var_hold == 2)
//    {
//      leds_enabled(RGB_LED_0, COLOR_BLUE, true);
//    }
 Si1133_send_force();
}
/***************************************************************************//**
 * @brief
 *Initializes the LEDs so that they may be able to change
 *
 * @details
 *Initializes var_hold, the static variable of the led color, to 0 and calls the routine to initialize the RGB LEDs
 *
 *
 * @note
 *Should be called in app_peripheral_setup
 *
 ******************************************************************************/
void led_switch(void)
{
  var_hold = 0;
  rgb_init();
  //call routine here

}
/***************************************************************************//**
 * @brief
 *Turns the BLUE LED ON/OFF depending on the result of the I2C sensing operation
 *
 * @details
 *If the result of the sensing operation is less than 20, we turn the Blue LED on. Otherwise, the Blue LED is off.
 *
 *
 * @note
 *Should be called after the read operation is complete
 *
 ******************************************************************************/
void scheduled_si1133_cb()
{
//  if(Si1133_pass_result() == EXPECTED_VALUE)
//    {
//      leds_enabled(RGB_LED_1, COLOR_GREEN, true);
//    }
//  if(Si1133_pass_result() != EXPECTED_VALUE)
//    {
//      leds_enabled(RGB_LED_1, COLOR_RED, true);
//    }
//
  unsigned int result = Si1133_pass_result();
  if(result < EXPECTED_VALUE_TWENTY)
    {
      leds_enabled(RGB_LED_1, COLOR_BLUE, true);
//      bool test = leuart_tx_busy(LEUART0);
//      if(test)
//        {
//          char str[26];
//          sprintf(str, "It's dark = %u\n", result);
//          ble_write(str);
//        }
    }
  if(result >= EXPECTED_VALUE_TWENTY)
    {
//      bool test = leuart_tx_busy(LEUART0);
//      if(test)
//        {
//          char str[26];
//          sprintf(str, "It's light outside = %u\n", result);
//          ble_write(str);
//        }
      leds_enabled(RGB_LED_1, COLOR_BLUE, false);
    }

}

/***************************************************************************//**
 * @brief
 *Calls ble_test() (when BLE_TEST_ENABLED is uncommented) and calls ble_write() and starts the letimer
 *
 * @details
 *If BLE_TEST_ENABLED is uncommented, this function calls ble_test and asserts the resulting boolean value. Followed by a necessary timer delay.
 *The function then calls ble_write() and then letimer_start.
 *
 *
 * @note
 *Should be called at the end of app_peripheral setup is complete
 *
 ******************************************************************************/
void scheduled_boot_up_cd(void)
{
#ifdef BLE_TEST_ENABLED
  bool isTrue = ble_test("NameChanged");
  EFM_ASSERT(isTrue);
  timer_delay(TWO_NUM);
#endif
  ble_write("\nHello World\n");
  letimer_start(LETIMER0, true);  //This command will initiate the start of the LETIMER0
  return;
}
/***************************************************************************//**
 * @brief
 *The function for when the tx operation is finished
 *
 * @details
 *currently does nothing.
 *
 *
 * @note
 *not called yet.
 *
 ******************************************************************************/
void scheduled_ble_tx_done_cd(void)
{

  return;
}
/***************************************************************************//**
 * @brief
 *The function for when a spi read operation is finished
 *
 * @details
 *This function obtains the z value from the accelerometer and, depending on its value and the
 *value of the static bool isFlipped, a message is displayed and the LED is turned on/off.
 *
 *
 * @note
 *Called when icm_read_print(ICM_DONE_CB) is called at the end of scheduled_letimer0_uf_cb().
 *
 ******************************************************************************/
void scheduled_icm_done_cb()
{
  uint16_t z_val = 0;
  z_val = icm_get_z_value();
  short acc_z = 0;
  acc_z = (short) z_val;
  int final_z = 0;
  final_z = (int) acc_z;
  if((final_z <= 0) && (isFlipped == false))
    {
      ble_write("\nHelp! Turn me over!\n\n");
      isFlipped = true;
      leds_enabled(RGB_LED_2, COLOR_GREEN, true);
    }
  else if((final_z > 0) && (isFlipped == true))
    {
      ble_write("\nThank you!\n\n");
      isFlipped = false;
      leds_enabled(RGB_LED_2, COLOR_GREEN, false);
    }
}
