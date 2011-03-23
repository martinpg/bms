#include <p18cxxx.h>
#include <timers.h>

/********************************************************************
*    Function Name:  CloseTimer5                                    *
*    Return Value:   void                                           *
*    Parameters:     void                                           *
*    Description:    This routine disables the Timer5 and the       *
*                    interrupt.                                     *
********************************************************************/

#if defined (TMR_V5) 

void CloseTimer5(void)
{
  T5CONbits.TMR5ON = 0;  // Disable Timer5
  PIE3bits.TMR5IE = 0;   // Disable Timer5 overflow interrupts
}

#elif defined (TMR_V7) || defined (TMR_V7_1)

void CloseTimer5(void)
{
  T5CONbits.TMR5ON = 0;  // Disable Timer5
  PIE5bits.TMR5IE = 0;   // Disable Timer5 overflow interrupts
}

#endif
