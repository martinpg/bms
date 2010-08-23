#include <p18cxxx.h>
#include <compare.h>


/********************************************************************
*    Function Name:   CloseECompare2                               *
*    Return Value:    void                                          *
*    Parameters:      void                                          *
*    Description:     This routine disables the capture interrupt.  *
********************************************************************/
#if defined (ECC_V8) || defined (ECC_V8_1)

void CloseECompare2(void)
{
#if defined(CC4_IO_V2)
   PIE2bits.CCP2IE = 0;    // Disable the interrupt
#else  
   PIE3bits.CCP2IE = 0;    // Disable the interrupt
#endif
   ECCP2CON=0x00;           // Reset the CCP module
}

#endif
