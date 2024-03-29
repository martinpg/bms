#include <p18cxxx.h>
#include <timers.h>


/********************************************************************
*    Function Name:  CloseTimer7                                   *
*    Return Value:   void                                           *
*    Parameters:     void                                           *
*    Description:    This routine disables Timer7 and interrupt.    *
********************************************************************/
#if defined (TMR_V7)
void CloseTimer7(void)
{
  T7CONbits.TMR7ON = 0;  // Disable Timer7
  PIE5bits.TMR7IE = 0;   // Disable Timer7 overflow interrupts 
}
#endif
