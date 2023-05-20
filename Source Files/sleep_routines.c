/**************************************************************************
* @file sleep_routines.c
* @author Noah Schwartz
* @date 9/23/21
* @brief Defines for functions that set up the sleep routines
***************************************************************************
* @section License
* <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
***************************************************************************
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*
* DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
* obligation to support this Software. Silicon Labs is providing the
* Software "AS IS", with no express or implied warranties of any kind,
* including, but not limited to, any implied warranties of merchantability
* or fitness for any particular purpose or warranties against infringement
* of any proprietary rights of a third party.
*
* Silicon Labs will not be liable for any consequential, incidental, or
* special damages, or any other relief, or for any claim by any third party,
* arising from your use of this Software.
*
*
**************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************
#include "sleep_routines.h"


//***********************************************************************************
// defined files
//***********************************************************************************
#define   ZERO_NUM    0;

//***********************************************************************************
// Private variables
//***********************************************************************************
static int lowest_energy_mode[MAX_ENERGY_MODES];

//***********************************************************************************
// Private functions
//***********************************************************************************

//***********************************************************************************
// Global functions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *Initializes the sleep routines functionality
 *
 * @details
 *"Initialize the sleep_routines static/private array, lowest_energy_mode[], to all zeroes"
 *
 * @note
 *Called when app_peripheral_setup is called

 ******************************************************************************/
void sleep_open(void)
{
  for(int p = 0; p < MAX_ENERGY_MODES; p++)
    {
      lowest_energy_mode[p] = ZERO_NUM;
    }
}
/***************************************************************************//**
 * @brief
 *Prevents the Mighty Gecko from going into sleep mode EM while a denoted peripheral is active
 *
 * @details
 * Utilized by a peripheral to prevent the Mighty Gecko from going into that sleep mode while
 * the peripheral is active. It will increment the associated array
 * element in lowest_energy_mode[] by one."
 *
 *
 * @note
 *Called in letimer_start() whenever the LETIMER is not running and enable is true
 *
 * @param[in]
 *EM: uint32_t value denoting the current energy mode the system is in.
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void sleep_block_mode(uint32_t EM)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  lowest_energy_mode[EM]++;
  CORE_EXIT_CRITICAL();
  EFM_ASSERT(lowest_energy_mode[EM] < 5);
}
/***************************************************************************//**
 * @brief
 *"Releases the processor from going into sleep mode with a peripheral that is no longer active"
 *
 * @details
 * "Utilized to release the processor from going into a
 * sleep mode with a peripheral that is no longer active.
 * It will decrement the associated array element in lowest_energy_mode[] by one."
 *
 * @note
 *Called in letimer_start() whenever the LETIMER is running and enable is false
 *
 * @param[in]
 *EM: uint32_t value denoting the current energy mode the system is in.
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/
void sleep_unblock_mode(uint32_t EM)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  lowest_energy_mode[EM]--;
  CORE_EXIT_CRITICAL();
  EFM_ASSERT(lowest_energy_mode[EM] >= 0);
}
/***************************************************************************//**
 * @brief
 * Puts the system into sleep mode when it is defined.
 *
 * @details
 *"Function that will enter the appropriate sleep Energy Mode
 * Function based on the first non-zero array element in lowest_energy_mode[]"
 *
 * @note
 *Called in main.c when no events are scheduled

 ******************************************************************************/
void enter_sleep(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  if(lowest_energy_mode[EM0] > 0)
    {
      CORE_EXIT_CRITICAL();
      return;
    }
  else if(lowest_energy_mode[EM1] > 0)
    {
      CORE_EXIT_CRITICAL();
      return;
    }
  else if(lowest_energy_mode[EM2] > 0)
    {
      EMU_EnterEM1();
      CORE_EXIT_CRITICAL();
      return;
    }
  else if(lowest_energy_mode[EM3] > 0)
    {
      EMU_EnterEM2(true);
      CORE_EXIT_CRITICAL();
      return;
    }
  else
    {
      EMU_EnterEM3(true);
      CORE_EXIT_CRITICAL();
      return;
    }
  CORE_EXIT_CRITICAL();
  return;
}
/***************************************************************************//**
 * @brief
 * Gets the energy mode(s) that the system can't enter.
 *
 * @details
 * Function that returns which energy mode that the
 * current system cannot enter, the first non-zero array element in
 * lowest_energy_mode[]."
 *
 * @note
 *Called whenever the info about the energy mode(s) the system can't enter is needed
 *
 * @param[in]
 *none
 *
 *@param[out]
 *returns a uint32_t value that denotes which energy mode the current system can't enter.
 ******************************************************************************/
uint32_t current_block_energy_mode(void)
{
  for(int i = 0; i < MAX_ENERGY_MODES; i++)
    {
      if(i < MAX_ENERGY_MODES)
        {
          if(lowest_energy_mode[i] != 0)
            {
              return i;
            }
        }
    }
  return (MAX_ENERGY_MODES - 1);
}
