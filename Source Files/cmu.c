/**
 * @file cmu.c
 * @author Noah Schwartz
 * @date 9/9/21
 * @brief Sets up the clock tree
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// Private variables
//***********************************************************************************


//***********************************************************************************
// Private functions
//***********************************************************************************


//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *Establishes the clock tree
 *
 * @details
 *By default, LFRCO and LFXO are enabled. We need to disable them in order to
 *get the clock tree running. Additionally, the LF clock needs to be routed to the clock tree
 *and then the clock tree needs to be enabled. This set up allows the clock to properly run once cmu_open() is called.
 * @note
 *The clock tree must be enabled after the peripherals are set up
 *
 ******************************************************************************/

void cmu_open(void){

    CMU_ClockEnable(cmuClock_HFPER, true);

    // By default, LFRCO is enabled, disable the LFRCO oscillator
    // Disable the LFRCO oscillator
    // What is the enumeration required for LFRCO?
    // It can be found in the online HAL documentation
    CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);

    // Disable the LFXO oscillator
    // What is the enumeration required for LFXO?
    // It can be found in the online HAL documentation
    CMU_OscillatorEnable(cmuOsc_LFXO, true, false);

    // No requirement to enable the ULFRCO oscillator.  It is always enabled in EM0-4H1

    // Route LF clock to the LF clock tree
    // What is the enumeration required to placed the ULFRCO onto the proper clock branch?
    // It can be found in the online HAL documentation
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);    // routing ULFRCO to proper Low Freq clock tree

    // What is the proper enumeration to enable the clock tree onto the LE clock branches?
    // It can be found in the Assignment 2 documentation
    CMU_ClockEnable(cmuClock_CORELE, true);

    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);    // routing ULFRCO to proper Low Freq clock tree

}

