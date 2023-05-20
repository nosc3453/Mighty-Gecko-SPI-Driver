/**
 * @file scheduler.c
 * @author Noah Schwartz
 * @date 9/23/21
 * @brief Defines for functions that set up the scheduler
 *
 */

#include "scheduler.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define EVENT_MODE_ZERO    0;


//***********************************************************************************
// Private variables
//*
static unsigned int event_scheduled;

//***********************************************************************************
// Private functions
//*

/***************************************************************************//**
 * @brief
 *Initializes the scheduler functionality
 *
 * @details
 *"Opens the scheduler functionality by resetting the static / private variable event_scheduled to 0."
 *
 * @note
 *Called when app_peripheral_setup is called

 ******************************************************************************/
void scheduler_open(void) // Opens the scheduler functionality by resetting the static / private variable event_scheduled to 0.
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  event_scheduled = EVENT_MODE_ZERO;
  CORE_EXIT_CRITICAL();
}
/***************************************************************************//**
 * @brief
 *"Adds an event to be scheduled
 *
 * @details
 * "ORs a new event, the input argument, into the existing state of the private (static) variable event_scheduled."
 *
 * @note
 *Called whenever it is called in LETIMER0_IRQHandler()
 *
 * @param[in]
 *event: an input has a bit value that denotes the event to be called.
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/


void add_scheduled_event(uint32_t event) // ORs a new event, the input argument, into the existing state of the private (static) variable event_scheduled.
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  event_scheduled |= event;
  CORE_EXIT_CRITICAL();
}
/***************************************************************************//**
 * @brief
 *Removes the event that is scheduled
 *
 * @details
"Removes the event, the input argument, from the existing state of the private (static) variable event_scheduled."
 *
 * @note
 *Called in main.c when the event is about to be called
 *
 * @param[in]
 *event: an input has a bit value that denotes the event to be called.
 *
 *@param[out]
 *returns nothing
 ******************************************************************************/

void remove_scheduled_event(uint32_t event) //Removes the event, the input argument, from the existing state of the private (static) variable event_scheduled.
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  event_scheduled = event_scheduled & ~event;
  CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *Returns the events that are scheduled
 *
 * @details
 *"Returns the current state of the private (static) variable event_scheduled."
 *
 * @note
 *Called in main.c to check which events are scheduled and in app.c for asserts.
 *
 *@param[out]
 *returns event_scheduled, a uint32_t variable which holds the state of the private (static) variable event_scheduled
 ******************************************************************************/
uint32_t get_scheduled_events(void) //Returns the current state of the private (static) variable event_scheduled.
{
  return event_scheduled;
}
